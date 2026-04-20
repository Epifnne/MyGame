#pragma once

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include "AssetMetadata.h"

namespace Runtime {
namespace Resource {

class AssetDatabase {
public:
	bool Register(const AssetMetadata& metadata);
	bool Unregister(const std::string& guid);

	std::optional<AssetMetadata> FindByGuid(const std::string& guid) const;
	std::optional<AssetMetadata> FindByVirtualPath(const std::string& virtualPath) const;
	std::optional<std::string> ResolveGuidByVirtualPath(const std::string& virtualPath) const;

private:
	mutable std::mutex m_mutex;
	std::unordered_map<std::string, AssetMetadata> m_byGuid;
	std::unordered_map<std::string, std::string> m_guidByPath;
};

} // namespace Resource
} // namespace Runtime
