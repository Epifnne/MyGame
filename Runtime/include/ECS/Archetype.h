#pragma once

#include "Entity.h"
#include <map>
#include <memory>
#include <typeindex>
#include <type_traits>
#include <utility>
#include <vector>

namespace Runtime::ECS {

class IComponentColumn {
public:
    virtual ~IComponentColumn() = default;
    virtual std::unique_ptr<IComponentColumn> CloneEmpty() const = 0;
    virtual void Reserve(size_t capacity) = 0;
    virtual void AppendMove(IComponentColumn& source, size_t row) noexcept = 0;
    virtual void Erase(size_t row) noexcept = 0;
};

template<typename T>
class ComponentColumn final : public IComponentColumn {
    static_assert(std::is_same_v<T, std::remove_cvref_t<T>>, "Component types must be unqualified");
    static_assert(std::is_nothrow_move_constructible_v<T> && std::is_nothrow_move_assignable_v<T>,
        "Archetype components must support noexcept move construction and assignment");
public:
    std::unique_ptr<IComponentColumn> CloneEmpty() const override {
        return std::make_unique<ComponentColumn<T>>();
    }
    void Reserve(size_t capacity) override { m_data.reserve(capacity); }
    void AppendMove(IComponentColumn& source, size_t row) noexcept override {
        m_data.push_back(std::move(static_cast<ComponentColumn<T>&>(source).m_data[row]));
    }
    void Append(T&& value) noexcept { m_data.push_back(std::move(value)); }
    void Erase(size_t row) noexcept override {
        if (row != m_data.size() - 1) m_data[row] = std::move(m_data.back());
        m_data.pop_back();
    }
    T& Get(size_t row) { return m_data[row]; }
    const T& Get(size_t row) const { return m_data[row]; }

private:
    std::vector<T> m_data;
};

struct Archetype {
    using Signature = std::vector<std::type_index>;
    std::map<std::type_index, std::unique_ptr<IComponentColumn>> columns;
    std::vector<Entity> entities;

    void ReserveRow() {
        if (entities.size() < entities.capacity()) return;
        const auto capacity = entities.capacity() == 0 ? size_t{8} : entities.capacity() * 2;
        for (auto& [type, column] : columns) column->Reserve(capacity);
        entities.reserve(capacity);
    }

    template<typename T>
    bool Has() const { return columns.contains(std::type_index(typeid(T))); }

    template<typename T>
    ComponentColumn<T>& Column() {
        return static_cast<ComponentColumn<T>&>(*columns.at(std::type_index(typeid(T))));
    }
    template<typename T>
    const ComponentColumn<T>& Column() const {
        return static_cast<const ComponentColumn<T>&>(*columns.at(std::type_index(typeid(T))));
    }

    void Erase(size_t row) noexcept {
        for (auto& [type, column] : columns) column->Erase(row);
        entities[row] = entities.back();
        entities.pop_back();
    }
};

} // namespace Runtime::ECS
