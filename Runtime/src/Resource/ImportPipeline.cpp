#include "Resource/ImportPipeline.h"

#include <functional>
#include <iomanip>
#include <sstream>

namespace Runtime {
namespace Resource {

std::string ImportPipeline::BuildGuid(const std::string& sourcePath, const std::string& resourceType) {
    const std::string key = sourcePath + "|" + resourceType;
    const size_t hash = std::hash<std::string>{}(key);

    std::ostringstream oss;
    oss << std::hex << std::setw(16) << std::setfill('0') << hash;
    return oss.str();
}

AssetMetadata ImportPipeline::BuildMetadata(const std::string& sourcePath,
                                            const std::string& virtualPath,
                                            const std::string& resourceType,
                                            uint32_t version) const {
    AssetMetadata metadata;
    metadata.guid = BuildGuid(sourcePath, resourceType);
    metadata.sourcePath = sourcePath;
    metadata.virtualPath = virtualPath;
    metadata.type = resourceType;
    metadata.version = version;
    return metadata;
}

} // namespace Resource
} // namespace Runtime
