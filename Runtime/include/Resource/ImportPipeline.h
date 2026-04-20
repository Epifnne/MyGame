#pragma once

#include <string>

#include "AssetMetadata.h"

namespace Runtime {
namespace Resource {

class ImportPipeline {
public:
	static std::string BuildGuid(const std::string& sourcePath, const std::string& resourceType);

	AssetMetadata BuildMetadata(const std::string& sourcePath,
								const std::string& virtualPath,
								const std::string& resourceType,
								uint32_t version = 1) const;
};

} // namespace Resource
} // namespace Runtime
