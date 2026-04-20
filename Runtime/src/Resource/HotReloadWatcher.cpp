#include "Resource/HotReloadWatcher.h"

namespace Runtime {
namespace Resource {

bool HotReloadWatcher::Watch(const std::string& absolutePath) {
    std::error_code ec;
    if (!std::filesystem::exists(absolutePath, ec)) {
        return false;
    }

    const auto fileTime = std::filesystem::last_write_time(absolutePath, ec);
    if (ec) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_fileTimes[absolutePath] = fileTime;
    return true;
}

bool HotReloadWatcher::Unwatch(const std::string& absolutePath) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_fileTimes.erase(absolutePath) > 0;
}

std::vector<std::string> HotReloadWatcher::PollChanges() {
    std::vector<std::string> changed;
    std::lock_guard<std::mutex> lock(m_mutex);

    for (auto& kv : m_fileTimes) {
        std::error_code ec;
        const auto now = std::filesystem::last_write_time(kv.first, ec);
        if (ec) {
            continue;
        }
        if (now != kv.second) {
            kv.second = now;
            changed.push_back(kv.first);
        }
    }

    return changed;
}

} // namespace Resource
} // namespace Runtime
