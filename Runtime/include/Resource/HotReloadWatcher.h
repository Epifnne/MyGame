#pragma once

#include <filesystem>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace Runtime {
namespace Resource {

class HotReloadWatcher {
public:
	bool Watch(const std::string& absolutePath);
	bool Unwatch(const std::string& absolutePath);
	std::vector<std::string> PollChanges();

private:
	mutable std::mutex m_mutex;
	std::unordered_map<std::string, std::filesystem::file_time_type> m_fileTimes;
};

} // namespace Resource
} // namespace Runtime
