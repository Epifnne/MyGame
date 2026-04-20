#include "Resource/TextureLoader.h"

#include "Resource/FileSystem.h"
#include "Resource/Resource.h"

namespace Runtime {
namespace Resource {

std::string TextureLoader::ResourceType() const {
    return "texture";
}

std::optional<std::vector<uint8_t>> TextureLoader::Read(const AssetMetadata& metadata, const FileSystem& fileSystem) const {
    auto bytes = fileSystem.ReadBinaryFile(metadata.sourcePath.empty() ? metadata.virtualPath : metadata.sourcePath);
    if (!bytes.has_value()) {
        return std::nullopt;
    }

    return bytes;
}

std::shared_ptr<Resource> TextureLoader::Decode(const AssetMetadata& metadata, std::vector<uint8_t> rawData) const {
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
