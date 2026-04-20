#include "Resource/MeshLoader.h"

#include "Resource/FileSystem.h"
#include "Resource/Resource.h"

namespace Runtime {
namespace Resource {

std::string MeshLoader::ResourceType() const {
    return "mesh";
}

std::optional<std::vector<uint8_t>> MeshLoader::Read(const AssetMetadata& metadata, const FileSystem& fileSystem) const {
    auto bytes = fileSystem.ReadBinaryFile(metadata.sourcePath.empty() ? metadata.virtualPath : metadata.sourcePath);
    if (!bytes.has_value()) {
        return std::nullopt;
    }

    return bytes;
}

std::shared_ptr<Resource> MeshLoader::Decode(const AssetMetadata& metadata, std::vector<uint8_t> rawData) const {
    auto resource = std::make_shared<BinaryResource>();
    resource->SetGuid(metadata.guid);
    resource->SetType(ResourceType());
    resource->SetSourcePath(metadata.sourcePath.empty() ? metadata.virtualPath : metadata.sourcePath);
    resource->SetData(std::move(rawData));
    resource->SetState(ResourceState::Loaded);
    return resource;
}

} // namespace Resource
} // namespace Runtime
