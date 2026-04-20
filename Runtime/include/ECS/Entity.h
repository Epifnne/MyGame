// Runtime/include/ECS/Entity.h
#pragma once

#include <cstdint>

namespace Runtime {
namespace ECS {

using Entity = uint32_t;
constexpr Entity NullEntity = 0;

} // namespace ECS
} // namespace Runtime
