// Runtime/include/ECS/Entity.h
#pragma once

#include <cstdint>

namespace Runtime {
namespace ECS {

using Entity = uint64_t;
constexpr Entity NullEntity = 0;

constexpr uint32_t EntityIndex(Entity entity) { return static_cast<uint32_t>(entity); }
constexpr uint32_t EntityGeneration(Entity entity) { return static_cast<uint32_t>(entity >> 32); }
constexpr Entity MakeEntity(uint32_t index, uint32_t generation) {
    return (static_cast<Entity>(generation) << 32) | index;
}

} // namespace ECS
} // namespace Runtime
