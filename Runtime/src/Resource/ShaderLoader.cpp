#include "Resource/ShaderLoader.h"

#include "Resource/FileSystem.h"
#include "Resource/Resource.h"

namespace Runtime {
namespace Resource {

std::string ShaderLoader::ResourceType() const {
    return "shader";
}

std::optional<std::vector<uint8_t>> ShaderLoader::Read(const AssetMetadata& metadata, const FileSystem& fileSystem) const {
    auto text = fileSystem.ReadTextFile(metadata.sourcePath.empty() ? metadata.virtualPath : metadata.sourcePath);
    if (!text.has_value()) {
        return std::nullopt;
    }

    return std::vector<uint8_t>(text->begin(), text->end());
}

std::shared_ptr<Resource> ShaderLoader::Decode(const AssetMetadata& metadata, std::vector<uint8_t> rawData) const {
    auto resource = std::make_shared<TextResource>();
    resource->SetGuid(metadata.guid);
    resource->SetType(ResourceType());
    resource->SetSourcePath(metadata.sourcePath.empty() ? metadata.virtualPath : metadata.sourcePath);
    resource->SetText(std::string(rawData.begin(), rawData.end()));
    resource->SetState(ResourceState::Loaded);
    return resource;
}

} // namespace Resource
} // namespace Runtime
