#pragma once

#include <cstdint>
#include <string>

namespace Runtime {
namespace Gameplay {

struct GameplayTag {
    uint32_t value = 0;

    [[nodiscard]] constexpr bool IsValid() const noexcept { return value != 0; }
    [[nodiscard]] constexpr bool operator==(const GameplayTag& other) const noexcept = default;
    [[nodiscard]] constexpr bool operator!=(const GameplayTag& other) const noexcept = default;
};

class GameplayTagRegistry {
public:
    [[nodiscard]] static GameplayTagRegistry& Instance() noexcept;

    GameplayTag RegisterTag(const std::string& name);
    [[nodiscard]] GameplayTag FindTag(const std::string& name) const;
    [[nodiscard]] std::string FindName(GameplayTag tag) const;

private:
    GameplayTagRegistry() = default;
};

} // namespace Gameplay
} // namespace Runtime
