#pragma once

#include "Archetype.h"
#include "EntityPool.h"
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <tuple>

namespace Runtime::ECS {

class Registry {
public:
    Registry() = default;
    Registry(const Registry&) = delete;
    Registry& operator=(const Registry&) = delete;
    Registry(Registry&&) = delete;
    Registry& operator=(Registry&&) = delete;

    [[nodiscard]] Entity CreateEntity() {
        CheckStructuralChange();
        auto& empty = GetOrCreateArchetype({});
        empty.ReserveRow();
        if (m_locations.size() <= m_entities.SlotCount()) m_locations.resize(m_entities.SlotCount() + 1);
        const Entity entity = m_entities.Create();
        m_locations[EntityIndex(entity)] = {&empty, empty.entities.size()};
        empty.entities.push_back(entity);
        return entity;
    }

    [[nodiscard]] bool IsAlive(Entity entity) const { return m_entities.IsAlive(entity); }

    void DestroyEntity(Entity entity) {
        if (!IsAlive(entity)) return;
        CheckStructuralChange();
        // Pool allocation, if needed, happens before touching the component rows.
        m_entities.Destroy(entity);
        EraseRow(m_locations[EntityIndex(entity)]);
        m_locations[EntityIndex(entity)] = {};
    }

    template<typename T, typename... Args>
    void AddComponent(Entity entity, Args&&... args) {
        auto& location = RequireAlive(entity);
        if (location.archetype->Has<T>()) return;
        CheckStructuralChange();
        T value(std::forward<Args>(args)...);
        auto signature = SignatureOf(*location.archetype);
        signature.push_back(std::type_index(typeid(T)));
        std::sort(signature.begin(), signature.end());
        Archetype& target = GetOrCreateArchetype(signature, location.archetype,
            std::type_index(typeid(T)), []() -> std::unique_ptr<IComponentColumn> {
                return std::make_unique<ComponentColumn<T>>();
            });
        target.ReserveRow();
        target.Column<T>().Append(std::move(value));
        MoveEntity(entity, location, target);
    }

    template<typename T>
    void RemoveComponent(Entity entity) {
        auto& location = RequireAlive(entity);
        if (!location.archetype->Has<T>()) return;
        CheckStructuralChange();
        auto signature = SignatureOf(*location.archetype);
        std::erase(signature, std::type_index(typeid(T)));
        auto& target = GetOrCreateArchetype(signature, location.archetype);
        target.ReserveRow();
        MoveEntity(entity, location, target);
    }

    template<typename T>
    [[nodiscard]] bool HasComponent(Entity entity) const {
        return IsAlive(entity) && m_locations[EntityIndex(entity)].archetype->Has<T>();
    }

    template<typename T>
    [[nodiscard]] T& GetComponent(Entity entity) {
        const auto& location = RequireAlive(entity);
        return location.archetype->Column<T>().Get(location.row);
    }

    template<typename T>
    [[nodiscard]] const T& GetComponent(Entity entity) const {
        const auto& location = RequireAlive(entity);
        return std::as_const(*location.archetype).Column<T>().Get(location.row);
    }

    template<typename... Ts>
    [[nodiscard]] std::vector<Entity> EntitiesWith() const {
        std::vector<Entity> result;
        for (const auto& [signature, archetype] : m_archetypes) {
            if ((archetype->Has<Ts>() && ...)) {
                result.insert(result.end(), archetype->entities.begin(), archetype->entities.end());
            }
        }
        return result;
    }

    template<typename RegistryType, typename... Ts>
    class QueryView {
    public:
        explicit QueryView(RegistryType& registry) : m_registry(registry) {}
        template<typename Fn>
        void Each(Fn&& fn) const { m_registry.template Each<Ts...>(std::forward<Fn>(fn)); }
    private:
        RegistryType& m_registry;
    };

    template<typename... Ts>
    [[nodiscard]] auto View() { return QueryView<Registry, Ts...>(*this); }
    template<typename... Ts>
    [[nodiscard]] auto View() const { return QueryView<const Registry, Ts...>(*this); }

private:
    struct Location {
        Archetype* archetype = nullptr;
        size_t row = 0;
    };
    class IterationGuard {
    public:
        explicit IterationGuard(const Registry& registry) : m_registry(registry) { ++m_registry.m_iterationDepth; }
        ~IterationGuard() { --m_registry.m_iterationDepth; }
    private:
        const Registry& m_registry;
    };

    template<typename... Ts, typename Fn>
    void Each(Fn&& fn) {
        IterationGuard guard(*this);
        for (auto& [signature, archetype] : m_archetypes) {
            if (!(archetype->Has<Ts>() && ...)) continue;
            auto columns = std::tie(archetype->Column<Ts>()...);
            for (size_t row = 0; row < archetype->entities.size(); ++row) {
                std::apply([&](auto&... column) {
                    std::invoke(fn, archetype->entities[row], column.Get(row)...);
                }, columns);
            }
        }
    }
    template<typename... Ts, typename Fn>
    void Each(Fn&& fn) const {
        IterationGuard guard(*this);
        for (const auto& [signature, archetype] : m_archetypes) {
            const auto& storage = std::as_const(*archetype);
            if (!(storage.Has<Ts>() && ...)) continue;
            auto columns = std::tie(storage.Column<Ts>()...);
            for (size_t row = 0; row < storage.entities.size(); ++row) {
                std::apply([&](const auto&... column) {
                    std::invoke(fn, storage.entities[row], column.Get(row)...);
                }, columns);
            }
        }
    }

    void CheckStructuralChange() const {
        if (m_iterationDepth != 0) throw std::logic_error("Structural changes are forbidden during View::Each");
    }
    const Location& RequireAlive(Entity entity) const {
        if (!IsAlive(entity)) throw std::invalid_argument("Invalid or stale entity handle");
        return m_locations[EntityIndex(entity)];
    }
    Location& RequireAlive(Entity entity) {
        if (!IsAlive(entity)) throw std::invalid_argument("Invalid or stale entity handle");
        return m_locations[EntityIndex(entity)];
    }
    static Archetype::Signature SignatureOf(const Archetype& archetype) {
        Archetype::Signature signature;
        for (const auto& [type, column] : archetype.columns) signature.push_back(type);
        return signature;
    }
    Archetype& GetOrCreateArchetype(const Archetype::Signature& signature,
        const Archetype* source = nullptr, std::type_index addedType = typeid(void),
        std::unique_ptr<IComponentColumn> (*makeColumn)() = nullptr) {
        if (auto it = m_archetypes.find(signature); it != m_archetypes.end()) return *it->second;
        auto archetype = std::make_unique<Archetype>();
        for (auto type : signature) {
            archetype->columns.emplace(type,
                type == addedType ? makeColumn() : source->columns.at(type)->CloneEmpty());
        }
        auto* result = archetype.get();
        m_archetypes.emplace(signature, std::move(archetype));
        return *result;
    }
    void EraseRow(const Location& location) noexcept {
        auto& archetype = *location.archetype;
        const Entity moved = archetype.entities.back();
        archetype.Erase(location.row);
        if (location.row < archetype.entities.size()) m_locations[EntityIndex(moved)].row = location.row;
    }
    void MoveEntity(Entity entity, Location& location, Archetype& target) noexcept {
        for (auto& [type, column] : target.columns) {
            if (auto it = location.archetype->columns.find(type); it != location.archetype->columns.end()) {
                column->AppendMove(*it->second, location.row);
            }
        }
        const size_t row = target.entities.size();
        target.entities.push_back(entity);
        EraseRow(location);
        location = {&target, row};
    }

    EntityPool m_entities;
    std::vector<Location> m_locations;
    std::map<Archetype::Signature, std::unique_ptr<Archetype>> m_archetypes;
    mutable size_t m_iterationDepth = 0;
};

} // namespace Runtime::ECS
