#include "Shared/Config/BuildConfig.h"

namespace Shared {
namespace Config {

const char* BuildConfig::EngineName() noexcept {
    return "MyGameEngine";
}

const char* BuildConfig::EngineVersion() noexcept {
    return "0.1.0";
}

} // namespace Config
} // namespace Shared
