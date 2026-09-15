#pragma once

namespace Shared {
namespace Config {

class BuildConfig {
public:
    [[nodiscard]] static const char* EngineName() noexcept;
    [[nodiscard]] static const char* EngineVersion() noexcept;
};

} // namespace Config
} // namespace Shared
