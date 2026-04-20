#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace Runtime {
namespace Resource {

struct AssetMetadata {
	std::string guid;
	std::string type;
	std::string virtualPath;
	std::string sourcePath;
	std::vector<std::string> dependencies;
	uint32_t version = 1;
};

} // namespace Resource
} // namespace Runtime
