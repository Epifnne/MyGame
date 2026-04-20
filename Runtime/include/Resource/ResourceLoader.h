#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "AssetMetadata.h"

namespace Runtime {
namespace Resource {

class FileSystem;
class Resource;

class ResourceLoader {
public:
	virtual ~ResourceLoader() = default;

	virtual std::string ResourceType() const = 0;
	virtual std::optional<std::vector<uint8_t>> Read(const AssetMetadata& metadata, const FileSystem& fileSystem) const = 0;
	virtual std::shared_ptr<Resource> Decode(const AssetMetadata& metadata, std::vector<uint8_t> rawData) const = 0;
	virtual bool Upload(const AssetMetadata& metadata, const std::shared_ptr<Resource>& resource) const;
};

} // namespace Resource
} // namespace Runtime
