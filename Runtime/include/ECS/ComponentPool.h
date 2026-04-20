// Runtime/include/ECS/ComponentPool.h
#pragma once

#include "Entity.h"
#include <vector>
#include <optional>
#include <algorithm>

namespace Runtime {
namespace ECS {

class IComponentPool {
public:
    virtual ~IComponentPool() = default;
    virtual void Erase(Entity e) = 0;
    virtual bool Has(Entity e) const = 0;
    virtual std::vector<Entity> Entities() const = 0;
};

template<typename T>
class ComponentPool {
public:
    template<typename... Args>
    void EmplaceConstruct(Entity e, Args&&... args) {
        if (e == NullEntity) return;
        if (e >= m_data.size()) m_data.resize(static_cast<size_t>(e) + 20);
        if (m_data[e].has_value()) return;
        m_data[e].emplace(std::forward<Args>(args)...);
        m_entities.push_back(e);
    }

    void Erase(Entity e) {
        if (e >= m_data.size() || !m_data[e].has_value()) return;
        m_data[e].reset();
        m_entities.erase(std::remove(m_entities.begin(), m_entities.end(), e), m_entities.end());
    }

    bool Has(Entity e) const { return e < m_data.size() && m_data[e].has_value(); }

    T& Get(Entity e) { return m_data.at(static_cast<size_t>(e)).value(); }
    const T& Get(Entity e) const { return m_data.at(static_cast<size_t>(e)).value(); }

    std::vector<Entity> Entities() const { return m_entities; }

private:
    std::vector<std::optional<T>> m_data; 
    std::vector<Entity> m_entities;

    // 若需要混合策略
    // sparse-set (dense/sparse)
    // std::vector<T> m_denseData;
    // std::vector<Entity> m_dense;
    // std::unordered_map<Entity, size_t> m_sparse;
};

template<typename T>
class ComponentPoolWrapper : public IComponentPool {
public:
    template<typename... Args>
    void EmplaceConstruct(Entity e, Args&&... args) { m_pool.EmplaceConstruct(e, std::forward<Args>(args)...); }
    void Erase(Entity e) override { m_pool.Erase(e); }
    bool Has(Entity e) const override { return m_pool.Has(e); }
    T& Get(Entity e) { return m_pool.Get(e); }
    const T& Get(Entity e) const { return m_pool.Get(e); }
    std::vector<Entity> Entities() const override { return m_pool.Entities(); }

private:
    ComponentPool<T> m_pool;
};

} // namespace ECS
} // namespace Runtime
