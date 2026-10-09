// Phase 7 acceptance gates: chained forward-star physics islands.
//
// - Every dynamic body belongs to at most one island.
// - Bodies sharing a static ground stay in independent islands (static bodies
//   are boundary nodes, never propagation nodes).
// - Trigger contacts never form edges.
// - A contactless dynamic body forms a single-body island.
// - The forward star grows as O(V + E): one head entry per dynamic body,
//   two edges per dynamic-dynamic contact, one per dynamic-static contact.
// - Output is stable-sorted (islands by smallest body id, bodies/contacts
//   ascending) and identical across repeated builds on reused capacity.
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <numeric>
#include <vector>

#include <glm/glm.hpp>

#include "Physics/CollisionShape.h"
#include "Physics/PhysicsIsland.h"
#include "Physics/PhysicsWorld.h"

namespace {

using Runtime::Physics::BoxShape;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::IslandBuildStats;
using Runtime::Physics::IslandContact;
using Runtime::Physics::PhysicsIsland;
using Runtime::Physics::PhysicsIslandBuilder;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBodyDesc;

constexpr uint32_t kStaticGround = 1000;

// Flatten islands into (body -> island) and (contact -> island) ownership
// vectors for membership assertions.
void CheckPartition(
    const std::vector<PhysicsIsland>& islands,
    uint32_t dynamicBodyCount,
    std::size_t expectedOwnedContacts) {
    std::vector<int> bodyOwner(dynamicBodyCount + 1, -1);
    std::vector<int> contactOwner(64, -1);
    std::size_t ownedContacts = 0;
    for (std::size_t islandIndex = 0; islandIndex < islands.size(); ++islandIndex) {
        const PhysicsIsland& island = islands[islandIndex];
        EXPECT_TRUE(std::is_sorted(island.bodies.begin(), island.bodies.end()));
        EXPECT_TRUE(std::is_sorted(island.contacts.begin(), island.contacts.end()));
        for (const uint32_t body : island.bodies) {
            ASSERT_LE(body, dynamicBodyCount);
            EXPECT_EQ(bodyOwner[body], -1) << "body " << body << " in two islands";
            bodyOwner[body] = static_cast<int>(islandIndex);
        }
        for (const uint32_t contact : island.contacts) {
            ASSERT_LT(contact, contactOwner.size());
            EXPECT_EQ(contactOwner[contact], -1) << "contact owned by two islands";
            contactOwner[contact] = static_cast<int>(islandIndex);
            ++ownedContacts;
        }
        // Island ordering: sorted by smallest body id.
        if (islandIndex > 0) {
            EXPECT_LT(islands[islandIndex - 1].bodies.front(), island.bodies.front());
        }
    }
    // Every dynamic body is covered by exactly one island.
    for (uint32_t body = 1; body <= dynamicBodyCount; ++body) {
        EXPECT_NE(bodyOwner[body], -1) << "body " << body << " in no island";
    }
    EXPECT_EQ(ownedContacts, expectedOwnedContacts);
}

TEST(PhysicsIsland, DynamicChainContactsMergeIntoOneIsland) {
    PhysicsIslandBuilder builder;
    const std::vector<uint32_t> bodies = {1, 2, 3};
    const std::vector<IslandContact> contacts = {
        {1, 2, false},
        {2, 3, false},
    };
    const std::vector<PhysicsIsland>& islands = builder.Build(bodies, contacts);
    ASSERT_EQ(islands.size(), 1u);
    EXPECT_EQ(islands[0].bodies, (std::vector<uint32_t>{1, 2, 3}));
    EXPECT_EQ(islands[0].contacts, (std::vector<uint32_t>{0, 1}));
    CheckPartition(islands, 3, 2);
    EXPECT_EQ(builder.LastStats().maxIslandBodyCount, 3u);
}

TEST(PhysicsIsland, ContactsFollowPairKeysEvenWhenInputIsUnsorted) {
    PhysicsIslandBuilder builder;
    const std::vector<IslandContact> contacts = {
        {3, 2, false},
        {2, 1, false},
        {3, 1, false},
    };
    const std::vector<PhysicsIsland>& islands = builder.Build({1, 2, 3}, contacts);
    ASSERT_EQ(islands.size(), 1u);
    EXPECT_EQ(islands[0].contacts, (std::vector<uint32_t>{1, 2, 0}));
}

TEST(PhysicsIsland, SharedStaticGroundKeepsIslandsIndependent) {
    PhysicsIslandBuilder builder;
    const std::vector<uint32_t> bodies = {1, 2};
    const std::vector<IslandContact> contacts = {
        {1, kStaticGround, false},
        {2, kStaticGround, false},
    };
    const std::vector<PhysicsIsland>& islands = builder.Build(bodies, contacts);
    ASSERT_EQ(islands.size(), 2u);
    EXPECT_EQ(islands[0].bodies, (std::vector<uint32_t>{1}));
    EXPECT_EQ(islands[0].contacts, (std::vector<uint32_t>{0}));
    EXPECT_EQ(islands[1].bodies, (std::vector<uint32_t>{2}));
    EXPECT_EQ(islands[1].contacts, (std::vector<uint32_t>{1}));
    // Static ground never appears as an island member.
    CheckPartition(islands, 2, 2);
}

TEST(PhysicsIsland, TriggerContactDoesNotFormEdge) {
    PhysicsIslandBuilder builder;
    const std::vector<uint32_t> bodies = {1, 2};
    const std::vector<IslandContact> contacts = {
        {1, 2, true},
    };
    const std::vector<PhysicsIsland>& islands = builder.Build(bodies, contacts);
    ASSERT_EQ(islands.size(), 2u);
    EXPECT_TRUE(islands[0].contacts.empty());
    EXPECT_TRUE(islands[1].contacts.empty());
    EXPECT_EQ(builder.LastStats().edgeCount, 0u);
    CheckPartition(islands, 2, 0);
}

TEST(PhysicsIsland, ContactlessDynamicBodyFormsSingleBodyIsland) {
    PhysicsIslandBuilder builder;
    const std::vector<uint32_t> bodies = {1, 2, 3};
    const std::vector<IslandContact> contacts;
    const std::vector<PhysicsIsland>& islands = builder.Build(bodies, contacts);
    ASSERT_EQ(islands.size(), 3u);
    for (std::size_t index = 0; index < islands.size(); ++index) {
        EXPECT_EQ(islands[index].bodies,
                  (std::vector<uint32_t>{static_cast<uint32_t>(index + 1)}));
        EXPECT_TRUE(islands[index].contacts.empty());
    }
    CheckPartition(islands, 3, 0);
}

TEST(PhysicsIsland, ForwardStarGrowsLinearlyWithNodesAndEdges) {
    PhysicsIslandBuilder builder;
    const auto edgeCountFor = [](std::size_t bodyCount, std::size_t dynamicPairs,
                                 std::size_t staticContacts) {
        PhysicsIslandBuilder localBuilder;
        std::vector<uint32_t> bodies(bodyCount);
        std::iota(bodies.begin(), bodies.end(), 1u);
        std::vector<IslandContact> contacts;
        for (std::size_t index = 0; index < dynamicPairs; ++index) {
            contacts.push_back({static_cast<uint32_t>(index + 1),
                                static_cast<uint32_t>(index + 2), false});
        }
        for (std::size_t index = 0; index < staticContacts; ++index) {
            contacts.push_back({static_cast<uint32_t>(index + 1), kStaticGround, false});
        }
        localBuilder.Build(bodies, contacts);
        return localBuilder.LastStats();
    };

    const IslandBuildStats small = edgeCountFor(8, 4, 3);
    EXPECT_EQ(small.nodeCount, 8u);
    // 2 edges per dynamic-dynamic contact, 1 per dynamic-static contact.
    EXPECT_EQ(small.edgeCount, 2u * 4u + 3u);

    // Doubling the graph doubles the forward-star storage: O(V + E).
    const IslandBuildStats doubled = edgeCountFor(16, 8, 6);
    EXPECT_EQ(doubled.nodeCount, 2u * small.nodeCount);
    EXPECT_EQ(doubled.edgeCount, 2u * small.edgeCount);
}

TEST(PhysicsIsland, StaticStaticAndDanglingContactsAreIgnored) {
    PhysicsIslandBuilder builder;
    const std::vector<uint32_t> bodies = {1};
    const std::vector<IslandContact> contacts = {
        {kStaticGround, kStaticGround + 1, false},
        {kStaticGround, kStaticGround + 2, true},
    };
    const std::vector<PhysicsIsland>& islands = builder.Build(bodies, contacts);
    ASSERT_EQ(islands.size(), 1u);
    EXPECT_TRUE(islands[0].contacts.empty());
    EXPECT_EQ(builder.LastStats().edgeCount, 0u);
}

TEST(PhysicsIsland, OutputIsStableSortedRegardlessOfInputOrder) {
    PhysicsIslandBuilder builder;
    const std::vector<uint32_t> bodies = {5, 1, 4, 2, 3};
    const std::vector<IslandContact> contacts = {
        {4, 5, false},
        {1, 2, false},
    };
    const std::vector<PhysicsIsland>& islands = builder.Build(bodies, contacts);
    ASSERT_EQ(islands.size(), 3u);
    EXPECT_EQ(islands[0].bodies, (std::vector<uint32_t>{1, 2}));
    EXPECT_EQ(islands[1].bodies, (std::vector<uint32_t>{3}));
    EXPECT_EQ(islands[2].bodies, (std::vector<uint32_t>{4, 5}));
    CheckPartition(islands, 5, 2);
}

TEST(PhysicsIsland, RepeatedBuildsOnReusedCapacityAreIdentical) {
    PhysicsIslandBuilder builder;
    const std::vector<uint32_t> bodies = {1, 2, 3, 4, 5, 6};
    const std::vector<IslandContact> contacts = {
        {1, 2, false},
        {2, 3, false},
        {5, kStaticGround, false},
        {4, 5, true},
    };
    const std::vector<PhysicsIsland> first = builder.Build(bodies, contacts);
    // Shrink, then rebuild the original input: output must be identical.
    builder.Build({1}, {});
    const std::vector<PhysicsIsland> second = builder.Build(bodies, contacts);
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t index = 0; index < first.size(); ++index) {
        EXPECT_EQ(first[index].bodies, second[index].bodies);
        EXPECT_EQ(first[index].contacts, second[index].contacts);
    }
}

// Integration through PhysicsWorld: two separated stacks on one shared static
// ground stay two islands; a contactless falling body adds a single-body
// island; the stats snapshot reports both.
TEST(PhysicsIsland, WorldReportsStableIslandStats) {
    auto world = std::make_unique<PhysicsWorld>();
    world->SetFixedTimeStep(1.0f / 60.0f);
    world->SetContinuousCollisionEnabled(false);

    // Dynamic bodies are created first so their ids stay 1..7 (CheckPartition
    // assumes dynamic ids start at 1); the static ground comes last.
    const auto shape = std::make_shared<BoxShape>(glm::vec3(0.45f));
    const auto addBox = [&](const glm::vec3& position) {
        RigidBodyDesc body;
        body.position = position;
        body.inertiaTensorDiagonal = glm::vec3(0.135f);
        const uint32_t id = world->CreateRigidBody(body);
        ColliderDesc collider;
        collider.shape = shape;
        collider.material.dynamicFriction = 0.55f;
        collider.material.staticFriction = 0.65f;
        EXPECT_TRUE(world->AttachCollider(id, collider));
        return id;
    };

    // Two stacks of three, far enough apart to never touch each other.
    for (int level = 0; level < 3; ++level) {
        addBox(glm::vec3(0.0f, 0.55f + static_cast<float>(level) * 0.95f, 0.0f));
        addBox(glm::vec3(10.0f, 0.55f + static_cast<float>(level) * 0.95f, 0.0f));
    }
    // One contactless dynamic body high above the scene.
    addBox(glm::vec3(0.0f, 50.0f, 0.0f));

    RigidBodyDesc ground;
    ground.position = glm::vec3(0.0f, -0.5f, 0.0f);
    ground.isStatic = true;
    ground.useGravity = false;
    const uint32_t groundId = world->CreateRigidBody(ground);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<BoxShape>(glm::vec3(40.0f, 0.5f, 40.0f));
    groundCollider.material.dynamicFriction = 0.6f;
    groundCollider.material.staticFriction = 0.7f;
    ASSERT_TRUE(world->AttachCollider(groundId, groundCollider));

    for (int frame = 0; frame < 30; ++frame) {
        world->Step(1.0f / 60.0f);
    }

    const Runtime::Physics::PhysicsStepStats& stats = world->LastStepStats();
    EXPECT_EQ(stats.islandCount, 3u);
    EXPECT_EQ(stats.islandMaxBodyCount, 3u);

    const std::vector<PhysicsIsland>& islands = world->LastIslands();
    ASSERT_EQ(islands.size(), 3u);
    CheckPartition(islands, 7, islands[0].contacts.size() + islands[1].contacts.size());
    // The two stacks own their contacts; the falling body owns none.
    std::size_t islandWithNoContacts = 0;
    for (const PhysicsIsland& island : islands) {
        if (island.contacts.empty()) {
            ++islandWithNoContacts;
            EXPECT_EQ(island.bodies.size(), 1u);
        }
    }
    EXPECT_EQ(islandWithNoContacts, 1u);
}

} // namespace
