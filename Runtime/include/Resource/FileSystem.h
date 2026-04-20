#pragma once

#include <cstdint>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace Runtime {
namespace Resource {

class FileSystem {
public:
	bool Mount(const std::string& mountName, const std::string& absoluteOrRelativeRoot);
	bool Unmount(const std::string& mountName);

	std::string ResolvePath(const std::string& virtualPath) const;
	bool Exists(const std::string& virtualPath) const;

	std::optional<std::string> ReadTextFile(const std::string& virtualPath) const;
	std::optional<std::vector<uint8_t>> ReadBinaryFile(const std::string& virtualPath) const;
	bool WriteTextFile(const std::string& virtualPath, const std::string& content);
	bool WriteBinaryFile(const std::string& virtualPath, const std::vector<uint8_t>& content);

private:
	mutable std::shared_mutex m_mutex;
	std::unordered_map<std::string, std::string> m_mounts;
};

} // namespace Resource
} // namespace Runtime
