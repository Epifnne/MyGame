#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <random>
#include <set>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>

#include "Physics/BroadPhase.h"
#include "Physics/BvhTree.h"
#include "Physics/CollisionShape.h"
#include "Physics/PhysicsWorld.h"

namespace {

using Runtime::Physics::BroadPhaseQueryCoverage;
using Runtime::Physics::BroadPhaseQueryResult;
using Runtime::Physics::Collider;
using Runtime::Physics::ColliderDesc;
using Runtime::Physics::DynamicBvhBroadPhase;
using Runtime::Physics::HybridBvhBroadPhase;
using Runtime::Physics::PairKey;
using Runtime::Physics::PhysicsWorld;
using Runtime::Physics::RigidBody;
using Runtime::Physics::RigidBodyDesc;

struct BroadPhaseScene {
    std::unordered_map<uint32_t, RigidBody> bodies;
    std::unordered_map<uint32_t, Collider> colliders;
    uint32_t nextId = 1;

    uint32_t AddBox(
        const glm::vec3& position,
        const glm::vec3& halfExtents,
        bool isStatic,
        uint32_t layer = 1u,
        uint32_t mask = 0xFFFFFFFFu) {
        RigidBodyDesc desc;
        desc.position = position;
        desc.isStatic = isStatic;
        desc.useGravity = false;
        const uint32_t id = nextId++;
        RigidBody body(desc);
        body.SetId(id);
        bodies.emplace(id, body);

        ColliderDesc colliderDesc;
        colliderDesc.shape = std::make_shared<Runtime::Physics::BoxShape>(halfExtents);
        colliderDesc.layer = layer;
        colliderDesc.mask = mask;
        Collider collider(colliderDesc);
        collider.SetBodyId(id);
        colliders.emplace(id, std::move(collider));
        return id;
    }

    void Remove(uint32_t id) {
        bodies.erase(id);
        colliders.erase(id);
    }

    Runtime::Physics::AABB TightAabb(uint32_t id) const {
        Runtime::Physics::ShapeTransform transform;
        transform.position = bodies.at(id).Position();
        transform.orientation = bodies.at(id).Orientation();
        return colliders.at(id).ComputeAABB(transform);
    }
};

bool ContainsPair(const std::vector<PairKey>& pairs, uint32_t a, uint32_t b) {
    const PairKey key = Runtime::Physics::MakePairKey(a, b);
    return std::find(pairs.begin(), pairs.end(), key) != pairs.end();
}

void ExpectSortedUnique(const std::vector<PairKey>& pairs) {
    EXPECT_TRUE(std::is_sorted(pairs.begin(), pairs.end()));
    EXPECT_EQ(std::adjacent_find(pairs.begin(), pairs.end()), pairs.end());
}

} // namespace

// Phase 1 acceptance: with identical leaf AABBs and filters, the hybrid
// broad-phase must produce exactly the legacy candidate set, and a brute-force
// tight-AABB oracle must find no missed pair.
TEST(HybridBroadPhaseTest, MatchesLegacyAndBruteForceOracle) {
    BroadPhaseScene scene;
    std::mt19937 rng(12345u);
    std::uniform_real_distribution<float> positionDist(-10.0f, 10.0f);
    std::uniform_real_distribution<float> extentDist(0.2f, 1.2f);
    std::uniform_int_distribution<int> staticDist(0, 2);

    for (int i = 0; i < 48; ++i) {
        const glm::vec3 position(positionDist(rng), positionDist(rng), positionDist(rng));
        const glm::vec3 halfExtents(extentDist(rng), extentDist(rng), extentDist(rng));
        const bool isStatic = staticDist(rng) == 0;
        // Every 7th body only collides within filter group 2.
        if (i % 7 == 0) {
            scene.AddBox(position, halfExtents, isStatic, 2u, 2u);
        } else {
            scene.AddBox(position, halfExtents, isStatic);
        }
    }

    HybridBvhBroadPhase hybrid;
    DynamicBvhBroadPhase legacy;
    const BroadPhaseQueryResult hybridResult = hybrid.ComputePairs(scene.colliders, scene.bodies);
    const BroadPhaseQueryResult legacyResult = legacy.ComputePairs(scene.colliders, scene.bodies);

    // Phase 9 contract: the standard query mode is ActiveDynamics (pairs
    // without an awake dynamic endpoint are not generated); with no sleeping
    // bodies it degenerates to a full-scene re-check.
    EXPECT_EQ(hybridResult.coverage, BroadPhaseQueryCoverage::ActiveDynamics);
    ExpectSortedUnique(hybridResult.pairs);
    ExpectSortedUnique(legacyResult.pairs);

    // Same fat AABB policy on the same frozen input must give identical sets.
    EXPECT_EQ(hybridResult.pairs, legacyResult.pairs);

    const auto& hybridCounts = hybrid.GetLeafCounts();
    ASSERT_TRUE(hybridCounts.tracked);
    EXPECT_EQ(hybridCounts.staticLeaves + hybridCounts.dynamicLeaves, scene.colliders.size());

    // Brute-force oracle over tight AABBs: no candidate may be missed.
    std::vector<uint32_t> ids;
    ids.reserve(scene.colliders.size());
    for (const auto& entry : scene.colliders) {
        ids.push_back(entry.first);
    }
    for (std::size_t i = 0; i < ids.size(); ++i) {
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const uint32_t idA = ids[i];
            const uint32_t idB = ids[j];
            if (!scene.colliders.at(idA).CanCollideWith(scene.colliders.at(idB))) {
                continue;
            }
            if (scene.bodies.at(idA).IsStatic() && scene.bodies.at(idB).IsStatic()) {
                continue;
            }
            if (!scene.TightAabb(idA).Intersects(scene.TightAabb(idB))) {
                continue;
            }
            EXPECT_TRUE(ContainsPair(hybridResult.pairs, idA, idB))
                << "missed oracle pair " << idA << " / " << idB;
        }
    }
}

// Phase 1 acceptance: moving a static body invalidates its static-tree leaf.
TEST(HybridBroadPhaseTest, StaticBodyMoveInvalidatesStaticTree) {
    BroadPhaseScene scene;
    const uint32_t groundId = scene.AddBox(glm::vec3(0.0f), glm::vec3(10.0f, 0.5f, 10.0f), true);
    const uint32_t boxId = scene.AddBox(glm::vec3(0.0f, 0.9f, 0.0f), glm::vec3(0.5f), false);

    HybridBvhBroadPhase hybrid;
    EXPECT_TRUE(ContainsPair(hybrid.ComputePairs(scene.colliders, scene.bodies).pairs, groundId, boxId));

    scene.bodies.at(groundId).SetPosition(glm::vec3(200.0f, 0.0f, 0.0f));
    const BroadPhaseQueryResult moved = hybrid.ComputePairs(scene.colliders, scene.bodies);
    EXPECT_FALSE(ContainsPair(moved.pairs, groundId, boxId));
    EXPECT_EQ(hybrid.GetLeafCounts().staticLeaves, 1u);
    EXPECT_EQ(hybrid.GetLeafCounts().dynamicLeaves, 1u);

    scene.bodies.at(groundId).SetPosition(glm::vec3(0.0f));
    EXPECT_TRUE(ContainsPair(hybrid.ComputePairs(scene.colliders, scene.bodies).pairs, groundId, boxId));
}

// Phase 1 acceptance: static/dynamic switches migrate leaves between trees
// without residuals, and static-static pairs are never generated.
TEST(HybridBroadPhaseTest, StaticDynamicSwitchMigratesLeaf) {
    BroadPhaseScene scene;
    const uint32_t aId = scene.AddBox(glm::vec3(0.0f), glm::vec3(0.5f), false);
    const uint32_t bId = scene.AddBox(glm::vec3(0.5f, 0.0f, 0.0f), glm::vec3(0.5f), false);

    HybridBvhBroadPhase hybrid;
    BroadPhaseQueryResult result = hybrid.ComputePairs(scene.colliders, scene.bodies);
    ASSERT_EQ(result.pairs.size(), 1u);
    EXPECT_TRUE(ContainsPair(result.pairs, aId, bId));

    scene.bodies.at(aId).SetStatic(true);
    result = hybrid.ComputePairs(scene.colliders, scene.bodies);
    ASSERT_EQ(result.pairs.size(), 1u);
    EXPECT_EQ(hybrid.GetLeafCounts().staticLeaves, 1u);
    EXPECT_EQ(hybrid.GetLeafCounts().dynamicLeaves, 1u);

    scene.bodies.at(bId).SetStatic(true);
    result = hybrid.ComputePairs(scene.colliders, scene.bodies);
    EXPECT_TRUE(result.pairs.empty());
    EXPECT_EQ(hybrid.GetLeafCounts().staticLeaves, 2u);
    EXPECT_EQ(hybrid.GetLeafCounts().dynamicLeaves, 0u);

    scene.bodies.at(aId).SetStatic(false);
    result = hybrid.ComputePairs(scene.colliders, scene.bodies);
    ASSERT_EQ(result.pairs.size(), 1u);
    EXPECT_TRUE(ContainsPair(result.pairs, aId, bId));
    ExpectSortedUnique(result.pairs);
}

// Phase 1 acceptance: several dynamics resting on one shared static ground
// produce one normalized pair each, with no duplicates.
TEST(HybridBroadPhaseTest, SharedStaticGroundHasNoDuplicatePairs) {
    BroadPhaseScene scene;
    const uint32_t groundId = scene.AddBox(glm::vec3(0.0f), glm::vec3(50.0f, 0.5f, 50.0f), true);
    std::vector<uint32_t> boxIds;
    for (int i = 0; i < 8; ++i) {
        boxIds.push_back(scene.AddBox(glm::vec3(i * 3.0f, 0.9f, 0.0f), glm::vec3(0.5f), false));
    }

    HybridBvhBroadPhase hybrid;
    const BroadPhaseQueryResult result = hybrid.ComputePairs(scene.colliders, scene.bodies);

    ASSERT_EQ(result.pairs.size(), boxIds.size());
    ExpectSortedUnique(result.pairs);
    for (const uint32_t boxId : boxIds) {
        EXPECT_TRUE(ContainsPair(result.pairs, groundId, boxId));
    }
    for (const PairKey& pair : result.pairs) {
        EXPECT_EQ(pair.bodyA, groundId);
    }
}

// Phase 1 acceptance: destroying a body removes its leaf from whichever tree
// held it and drops its pairs.
TEST(HybridBroadPhaseTest, DestroyedBodyLeavesNoResidualLeaf) {
    BroadPhaseScene scene;
    const uint32_t groundId = scene.AddBox(glm::vec3(0.0f), glm::vec3(10.0f, 0.5f, 10.0f), true);
    const uint32_t boxId = scene.AddBox(glm::vec3(0.0f, 0.9f, 0.0f), glm::vec3(0.5f), false);
    scene.AddBox(glm::vec3(100.0f, 0.0f, 0.0f), glm::vec3(0.5f), false);

    HybridBvhBroadPhase hybrid;
    BroadPhaseQueryResult result = hybrid.ComputePairs(scene.colliders, scene.bodies);
    EXPECT_TRUE(ContainsPair(result.pairs, groundId, boxId));
    EXPECT_EQ(hybrid.GetLeafCounts().dynamicLeaves, 2u);

    scene.Remove(boxId);
    result = hybrid.ComputePairs(scene.colliders, scene.bodies);
    EXPECT_TRUE(result.pairs.empty());
    EXPECT_EQ(hybrid.GetLeafCounts().staticLeaves, 1u);
    EXPECT_EQ(hybrid.GetLeafCounts().dynamicLeaves, 1u);
}

namespace {

// Collect the overlapping leaf-id multiset a tree currently reports.
std::vector<std::pair<uint32_t, uint32_t>> CollectPairs(Runtime::Physics::BvhTree& tree) {
    std::vector<std::pair<uint32_t, uint32_t>> pairs;
    tree.SelfQueryPairs([&](uint32_t a, uint32_t b) { pairs.emplace_back(a, b); });
    std::sort(pairs.begin(), pairs.end());
    return pairs;
}

} // namespace

// Refit semantics: a leaf whose tight AABB stays inside the fat AABB is
// refitted in place (no remove + reinsert), and queries stay correct.
TEST(BvhTreeTest, RefitKeepsLeafWhenTightAabbStaysInsideFat) {
    Runtime::Physics::BvhTree tree;
    tree.UpsertLeaf(1, Runtime::Physics::AABB{glm::vec3(-0.5f), glm::vec3(0.5f)});
    tree.UpsertLeaf(2, Runtime::Physics::AABB{glm::vec3(0.4f, -0.5f, -0.5f), glm::vec3(1.4f, 0.5f, 0.5f)});

    const uint32_t updatesBefore = tree.UpdatesSinceRebuild();
    const double totalBefore = tree.TotalNodeSurfaceArea();
    EXPECT_GT(totalBefore, 0.0);
    EXPECT_GT(tree.LeafSurfaceArea(), 0.0);
    EXPECT_LE(tree.LeafSurfaceArea(), totalBefore);

    // Small move: still inside the fat AABB -> pure refit, no reinsertion.
    tree.UpsertLeaf(1, Runtime::Physics::AABB{glm::vec3(-0.55f), glm::vec3(0.45f)});
    EXPECT_EQ(tree.UpdatesSinceRebuild(), updatesBefore);
    EXPECT_EQ(tree.LeafCount(), 2u);

    const auto pairs = CollectPairs(tree);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0], std::make_pair(1u, 2u));

    // Query metrics are populated by the query above.
    const auto metrics = tree.LastSelfQueryMetrics();
    EXPECT_GT(metrics.visitedNodes, 0u);
    EXPECT_EQ(metrics.candidateLeafPairs, 1u);
    EXPECT_DOUBLE_EQ(metrics.totalNodeSurfaceArea, tree.TotalNodeSurfaceArea());
    EXPECT_DOUBLE_EQ(metrics.leafSurfaceArea, tree.LeafSurfaceArea());
}

// Refit escalation: surface-area growth beyond the threshold upgrades the
// refit to a remove + reinsert, counting toward the rebuild budget.
TEST(BvhTreeTest, RefitEscalatesToReinsertOnLargeSurfaceGrowth) {
    Runtime::Physics::BvhTree tree;
    tree.SetRefitSurfaceAreaGrowthThreshold(0.01f); // tolerate ~1% growth
    tree.SetRebuildUpdateThreshold(0);              // disable auto rebuild
    tree.UpsertLeaf(1, Runtime::Physics::AABB{glm::vec3(-0.5f), glm::vec3(0.5f)});

    // Grow the box a lot while staying inside the fat AABB is impossible, so
    // shrink+shift to force a fat-AABB change; the growth gate escalates.
    tree.UpsertLeaf(1, Runtime::Physics::AABB{glm::vec3(-2.0f), glm::vec3(2.0f)});
    EXPECT_EQ(tree.UpdatesSinceRebuild(), 1u);

    // Disabling the growth check restores the pure-refit path.
    Runtime::Physics::BvhTree lenient;
    lenient.SetRefitSurfaceAreaGrowthThreshold(0.0f);
    lenient.SetRebuildUpdateThreshold(0);
    lenient.UpsertLeaf(1, Runtime::Physics::AABB{glm::vec3(-0.5f), glm::vec3(0.5f)});
    lenient.UpsertLeaf(1, Runtime::Physics::AABB{glm::vec3(-0.52f), glm::vec3(0.48f)});
    EXPECT_EQ(lenient.UpdatesSinceRebuild(), 0u);
}

// Rebuild semantics: enough reinsertions trigger a full rebuild that preserves
// the leaf set and overlap queries while resetting the chaos counter.
TEST(BvhTreeTest, RebuildPreservesLeavesAndResetsChaos) {
    Runtime::Physics::BvhTree tree;
    tree.SetRebuildUpdateThreshold(4);

    // Overlapping 4x4 grid: spacing 0.3 with 0.5-sized boxes overlap, and a
    // uniform +x shift of 0.2/0.4/0.6 keeps the overlap relation unchanged.
    const int leafCount = 16;
    const auto leafAabb = [](int i, int round) {
        const float x = static_cast<float>(i % 4) * 0.3f + 0.2f * static_cast<float>(round);
        const float y = static_cast<float>(i / 4) * 0.3f;
        return Runtime::Physics::AABB{glm::vec3(x, y, 0.0f), glm::vec3(x + 0.5f, y + 0.5f, 0.5f)};
    };
    for (int i = 0; i < leafCount; ++i) {
        tree.UpsertLeaf(static_cast<uint32_t>(i + 1), leafAabb(i, 0));
    }
    const auto pairsBefore = CollectPairs(tree);
    ASSERT_FALSE(pairsBefore.empty());

    // Jitter leaves to accumulate reinsertions until the rebuild threshold fires.
    for (int round = 1; round <= 3; ++round) {
        for (int i = 0; i < leafCount; ++i) {
            tree.UpsertLeaf(static_cast<uint32_t>(i + 1), leafAabb(i, round));
        }
    }

    // Threshold 4 with 48 reinserts: rebuild must have fired and reset chaos.
    EXPECT_LT(tree.UpdatesSinceRebuild(), 4u);
    EXPECT_EQ(tree.LeafCount(), static_cast<std::size_t>(leafCount));

    // The overlap relation of the final configuration is unchanged.
    const auto pairsAfter = CollectPairs(tree);
    EXPECT_EQ(pairsAfter.size(), pairsBefore.size());
    EXPECT_EQ(pairsAfter, pairsBefore);
}

// Phase 1 A/B gate: with the same frozen input, hybrid and legacy broad-phase
// worlds must agree on the per-step contact pair set and final state.
TEST(PhysicsWorldBroadPhaseTest, HybridAndLegacyProduceSameContacts) {
    const auto buildWorld = [](bool legacy) {
        auto world = std::make_unique<PhysicsWorld>();
        world->SetFixedTimeStep(1.0f / 120.0f);
        world->SetGravity(glm::vec3(0.0f, -9.81f, 0.0f));
        world->SetContinuousCollisionEnabled(false);
        world->SetLegacyBroadPhaseEnabled(legacy);

        RigidBodyDesc groundDesc;
        groundDesc.position = {0.0f, -0.5f, 0.0f};
        groundDesc.isStatic = true;
        groundDesc.useGravity = false;
        const uint32_t groundId = world->CreateRigidBody(groundDesc);
        ColliderDesc groundCollider;
        groundCollider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(5.0f, 0.5f, 5.0f));
        world->AttachCollider(groundId, groundCollider);

        for (int i = 0; i < 5; ++i) {
            RigidBodyDesc boxDesc;
            boxDesc.position = {i * 0.25f, 1.0f + i * 1.2f, 0.0f};
            boxDesc.mass = 1.0f;
            const uint32_t boxId = world->CreateRigidBody(boxDesc);
            ColliderDesc boxCollider;
            boxCollider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(0.5f));
            boxCollider.material.restitution = 0.1f;
            world->AttachCollider(boxId, boxCollider);
        }
        return world;
    };

    auto hybridWorld = buildWorld(false);
    auto legacyWorld = buildWorld(true);
    EXPECT_FALSE(hybridWorld->LegacyBroadPhaseEnabled());
    EXPECT_TRUE(legacyWorld->LegacyBroadPhaseEnabled());

    const auto contactPairSet = [](const PhysicsWorld& world) {
        std::set<uint64_t> keys;
        for (const auto& contact : world.Contacts()) {
            const uint32_t lo = std::min(contact.bodyA, contact.bodyB);
            const uint32_t hi = std::max(contact.bodyA, contact.bodyB);
            keys.insert((static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi));
        }
        return keys;
    };

    for (int frame = 0; frame < 240; ++frame) {
        hybridWorld->Step(1.0f / 120.0f);
        legacyWorld->Step(1.0f / 120.0f);
        EXPECT_EQ(contactPairSet(*hybridWorld), contactPairSet(*legacyWorld)) << "frame " << frame;
    }

    for (uint32_t id = 1; id <= 6; ++id) {
        const auto* hybridBody = hybridWorld->GetRigidBody(id);
        const auto* legacyBody = legacyWorld->GetRigidBody(id);
        ASSERT_NE(hybridBody, nullptr);
        ASSERT_NE(legacyBody, nullptr);
        for (int axis = 0; axis < 3; ++axis) {
            EXPECT_NEAR(hybridBody->Position()[axis], legacyBody->Position()[axis], 1e-4f)
                << "body " << id << " axis " << axis;
        }
    }
}

// Patch 6: step statistics read the actual leaf counts of the two BVH trees.
TEST(PhysicsWorldBroadPhaseTest, StatsReportActualTreeLeafCounts) {
    PhysicsWorld world;
    world.SetFixedTimeStep(1.0f / 120.0f);
    world.SetContinuousCollisionEnabled(false);

    RigidBodyDesc groundDesc;
    groundDesc.isStatic = true;
    groundDesc.useGravity = false;
    const uint32_t groundId = world.CreateRigidBody(groundDesc);
    ColliderDesc groundCollider;
    groundCollider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(5.0f, 0.5f, 5.0f));
    world.AttachCollider(groundId, groundCollider);

    uint32_t lastBoxId = 0;
    for (int i = 0; i < 3; ++i) {
        RigidBodyDesc boxDesc;
        boxDesc.position = {i * 4.0f, 2.0f, 0.0f};
        lastBoxId = world.CreateRigidBody(boxDesc);
        ColliderDesc boxCollider;
        boxCollider.shape = std::make_shared<Runtime::Physics::BoxShape>(glm::vec3(0.5f));
        world.AttachCollider(lastBoxId, boxCollider);
    }

    world.Step(1.0f / 120.0f);
    EXPECT_EQ(world.LastStepStats().staticBvhLeafCount, 1u);
    EXPECT_EQ(world.LastStepStats().dynamicBvhLeafCount, 3u);

    ASSERT_TRUE(world.DestroyRigidBody(lastBoxId));
    world.Step(1.0f / 120.0f);
    EXPECT_EQ(world.LastStepStats().staticBvhLeafCount, 1u);
    EXPECT_EQ(world.LastStepStats().dynamicBvhLeafCount, 2u);
}
