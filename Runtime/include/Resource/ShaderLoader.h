#pragma once

#include <string>

#include "ResourceLoader.h"

namespace Runtime {
namespace Resource {

class ShaderLoader final : public ResourceLoader {
public:
	std::string ResourceType() const override;
	std::optional<std::vector<uint8_t>> Read(const AssetMetadata& metadata, const FileSystem& fileSystem) const override;
	std::shared_ptr<Resource> Decode(const AssetMetadata& metadata, std::vector<uint8_t> rawData) const override;
};

} // namespace Resource
} // namespace Runtime
