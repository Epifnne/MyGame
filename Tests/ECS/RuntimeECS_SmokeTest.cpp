#include <gtest/gtest.h>
#include <memory>
#include <random>
#include <unordered_map>
#include <unordered_set>

#include "ECS/Component.h"
#include "ECS/Entity.h"
#include "ECS/World.h"

using Runtime::ECS::NullEntity;
using Runtime::ECS::World;
using Runtime::ECS::Components::Transform2D;

TEST(EcsWorldTest, AddGetAndDestroyComponent) {
    World world;

    const Runtime::ECS::Entity e = world.CreateEntity();
    EXPECT_NE(e, NullEntity);

    world.AddComponent<Transform2D>(e, Transform2D{1.0f, 2.0f});
    EXPECT_TRUE(world.HasComponent<Transform2D>(e));

    auto& t = world.RegistryRef().GetComponent<Transform2D>(e);
    EXPECT_FLOAT_EQ(t.x, 1.0f);
    EXPECT_FLOAT_EQ(t.y, 2.0f);

    world.DestroyEntity(e);
    EXPECT_FALSE(world.HasComponent<Transform2D>(e));
}

TEST(EcsWorldTest, NullEntityCannotReceiveComponent) {
    World world;

    EXPECT_THROW(world.AddComponent<Transform2D>(NullEntity, Transform2D{3.0f, 4.0f}),
        std::invalid_argument);
    EXPECT_FALSE(world.HasComponent<Transform2D>(NullEntity));
}

TEST(EcsWorldTest, ReusedIndexNeverRevivesStaleHandle) {
    World world;
    const auto old = world.CreateEntity();
    world.AddComponent<Transform2D>(old, Transform2D{1.0f, 2.0f});
    world.DestroyEntity(old);
    const auto replacement = world.CreateEntity();
    EXPECT_EQ(Runtime::ECS::EntityIndex(old), Runtime::ECS::EntityIndex(replacement));
    EXPECT_NE(old, replacement);
    EXPECT_FALSE(world.IsAlive(old));
    EXPECT_TRUE(world.IsAlive(replacement));
    world.AddComponent<Transform2D>(replacement, Transform2D{3.0f, 4.0f});
    EXPECT_THROW(world.AddComponent<Transform2D>(old), std::invalid_argument);
    EXPECT_THROW(world.RemoveComponent<Transform2D>(old), std::invalid_argument);
    EXPECT_THROW((void)world.RegistryRef().GetComponent<Transform2D>(old), std::invalid_argument);
    const auto& constRegistry = std::as_const(world.RegistryRef());
    EXPECT_THROW((void)constRegistry.GetComponent<Transform2D>(old), std::invalid_argument);
    world.DestroyEntity(old);
    EXPECT_TRUE(world.IsAlive(replacement));
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(replacement).x, 3.0f);
}

namespace {
struct Marker {};
struct Unregistered {};
struct Owner {
    std::unique_ptr<int> value;
    explicit Owner(int v) : value(std::make_unique<int>(v)) {}
};
struct alignas(64) Aligned {
    int value = 7;
};
struct ThrowingConstruction {
    ThrowingConstruction() { throw std::runtime_error("construction failed"); }
};
}

TEST(EcsWorldTest, MigrationPreservesComponentsAndRepairsSwapRows) {
    World world;
    const auto first = world.CreateEntity();
    const auto second = world.CreateEntity();
    const auto third = world.CreateEntity();
    world.AddComponent<Transform2D>(first, Transform2D{1.0f, 0.0f});
    world.AddComponent<Transform2D>(second, Transform2D{2.0f, 0.0f});
    world.AddComponent<Transform2D>(third, Transform2D{3.0f, 0.0f});
    world.AddComponent<Owner>(first, 42);
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(third).x, 3.0f);
    world.AddComponent<Aligned>(first);
    world.AddComponent<Marker>(first);
    world.RemoveComponent<Marker>(first);
    world.RemoveComponent<Aligned>(first);
    EXPECT_EQ(*world.RegistryRef().GetComponent<Owner>(first).value, 42);
    world.RemoveComponent<Owner>(first);
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(first).x, 1.0f);
    world.DestroyEntity(second);
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(first).x, 1.0f);
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(third).x, 3.0f);
    world.RemoveComponent<Transform2D>(third);
    EXPECT_FALSE(world.HasComponent<Transform2D>(third));
    EXPECT_TRUE(world.IsAlive(third));
}

TEST(EcsWorldTest, DuplicateAddAndAbsentRemovePreserveData) {
    World world;
    const auto entity = world.CreateEntity();
    world.AddComponent<Transform2D>(entity, Transform2D{5.0f, 6.0f});
    world.AddComponent<Transform2D>(entity, Transform2D{9.0f, 9.0f});
    world.RemoveComponent<Marker>(entity);
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(entity).x, 5.0f);
    EXPECT_THROW((void)world.RegistryRef().GetComponent<Marker>(entity), std::out_of_range);
}

TEST(EcsWorldTest, MultiComponentViewMatchesIntersectionAndConstAccess) {
    World world;
    EXPECT_TRUE((world.RegistryRef().EntitiesWith<Transform2D, Marker>().empty()));
    const auto partial = world.CreateEntity();
    const auto full = world.CreateEntity();
    const auto superset = world.CreateEntity();
    world.AddComponent<Transform2D>(partial);
    world.AddComponent<Marker>(full);
    world.AddComponent<Transform2D>(full);
    world.AddComponent<Transform2D>(superset);
    world.AddComponent<Marker>(superset);
    world.AddComponent<Owner>(superset, 2);
    std::unordered_set<Runtime::ECS::Entity> seen;
    world.View<Transform2D, Marker>().Each(
        [&](Runtime::ECS::Entity e, Transform2D& transform, Marker&) {
            seen.insert(e);
            transform.x = 8.0f;
        });
    EXPECT_EQ(seen, (std::unordered_set<Runtime::ECS::Entity>{full, superset}));
    int count = 0;
    std::as_const(world).View<Transform2D, Marker>().Each(
        [&](Runtime::ECS::Entity, const Transform2D& transform, const Marker&) {
            EXPECT_FLOAT_EQ(transform.x, 8.0f);
            ++count;
        });
    EXPECT_EQ(count, 2);
    world.View<Unregistered>().Each([](Runtime::ECS::Entity, Unregistered&) { FAIL(); });
    EXPECT_EQ(world.RegistryRef().EntitiesWith<>().size(), 3u);
}

TEST(EcsWorldTest, MatchingArchetypeStoresComponentsContiguously) {
    World world;
    for (int i = 0; i < 200; ++i) {
        const auto e = world.CreateEntity();
        world.AddComponent<Transform2D>(e, Transform2D{static_cast<float>(i), 0.0f});
        world.AddComponent<Aligned>(e);
    }
    const Transform2D* previous = nullptr;
    int count = 0;
    world.View<Transform2D, Aligned>().Each(
        [&](Runtime::ECS::Entity, Transform2D& transform, Aligned& aligned) {
            if (previous) EXPECT_EQ(&transform, previous + 1);
            previous = &transform;
            EXPECT_EQ(reinterpret_cast<uintptr_t>(&aligned) % alignof(Aligned), 0u);
            ++count;
        });
    EXPECT_EQ(count, 200);
}

TEST(EcsWorldTest, ViewRejectsStructuralMutationAndRestoresGuardAfterException) {
    World world;
    const auto e = world.CreateEntity();
    world.AddComponent<Transform2D>(e);
    EXPECT_THROW(world.View<Transform2D>().Each(
        [&](Runtime::ECS::Entity entity, Transform2D&) { world.AddComponent<Marker>(entity); }),
        std::logic_error);
    world.AddComponent<Marker>(e);
    world.View<Transform2D>().Each([&](Runtime::ECS::Entity entity, Transform2D&) {
        EXPECT_THROW(world.RemoveComponent<Marker>(entity), std::logic_error);
        EXPECT_THROW(world.DestroyEntity(entity), std::logic_error);
        EXPECT_THROW((void)world.CreateEntity(), std::logic_error);
        std::as_const(world).View<Marker>().Each([](Runtime::ECS::Entity, const Marker&) {});
    });
    world.DestroyEntity(e);
    EXPECT_FALSE(world.IsAlive(e));
}

TEST(EcsWorldTest, FailedConstructionDoesNotChangeEntity) {
    World world;
    const auto e = world.CreateEntity();
    world.AddComponent<Transform2D>(e, Transform2D{7.0f, 0.0f});
    EXPECT_THROW(world.AddComponent<ThrowingConstruction>(e), std::runtime_error);
    EXPECT_FALSE(world.HasComponent<ThrowingConstruction>(e));
    EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(e).x, 7.0f);
}

TEST(EcsWorldTest, RandomizedStructuralChangesMatchReferenceModel) {
    World world;
    using Entity = Runtime::ECS::Entity;
    struct State { bool transform = false; bool marker = false; float value = 0.0f; };
    std::unordered_map<Entity, State> states;
    std::vector<Entity> handles;
    std::mt19937 random(42);
    for (int i = 0; i < 5000; ++i) {
        const auto operation = random() % 6;
        if (handles.empty() || operation == 0) {
            const auto entity = world.CreateEntity();
            states.emplace(entity, State{});
            handles.push_back(entity);
        } else {
            const auto entity = handles[random() % handles.size()];
            if (!states.contains(entity)) {
                EXPECT_FALSE(world.IsAlive(entity));
                EXPECT_FALSE(world.HasComponent<Transform2D>(entity));
                continue;
            }
            auto& state = states.at(entity);
            switch (operation) {
            case 1:
                world.AddComponent<Transform2D>(entity, Transform2D{static_cast<float>(i), 0.0f});
                if (!state.transform) state.value = static_cast<float>(i);
                state.transform = true;
                break;
            case 2:
                world.RemoveComponent<Transform2D>(entity);
                state.transform = false;
                break;
            case 3:
                world.AddComponent<Marker>(entity);
                state.marker = true;
                break;
            case 4:
                world.RemoveComponent<Marker>(entity);
                state.marker = false;
                break;
            case 5:
                world.DestroyEntity(entity);
                states.erase(entity);
                break;
            }
        }
        size_t expectedCount = 0;
        for (const auto& [entity, state] : states) {
            EXPECT_TRUE(world.IsAlive(entity));
            EXPECT_EQ(world.HasComponent<Transform2D>(entity), state.transform);
            EXPECT_EQ(world.HasComponent<Marker>(entity), state.marker);
            if (state.transform) EXPECT_FLOAT_EQ(world.RegistryRef().GetComponent<Transform2D>(entity).x, state.value);
            if (state.transform && state.marker) ++expectedCount;
        }
        size_t actualCount = 0;
        world.View<Transform2D, Marker>().Each([&](Entity entity, Transform2D& transform, Marker&) {
            EXPECT_FLOAT_EQ(transform.x, states.at(entity).value);
            ++actualCount;
        });
        EXPECT_EQ(actualCount, expectedCount);
    }
}
