#include "Resource/AssetDatabase.h"

namespace Runtime {
namespace Resource {

bool AssetDatabase::Register(const AssetMetadata& metadata) {
    if (metadata.guid.empty() || metadata.virtualPath.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    auto existingPath = m_guidByPath.find(metadata.virtualPath);
    if (existingPath != m_guidByPath.end() && existingPath->second != metadata.guid) {
        return false;
    }

    m_byGuid[metadata.guid] = metadata;
    m_guidByPath[metadata.virtualPath] = metadata.guid;
    return true;
}

bool AssetDatabase::Unregister(const std::string& guid) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_byGuid.find(guid);
    if (it == m_byGuid.end()) {
        return false;
    }

    m_guidByPath.erase(it->second.virtualPath);
    m_byGuid.erase(it);
    return true;
}

std::optional<AssetMetadata> AssetDatabase::FindByGuid(const std::string& guid) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_byGuid.find(guid);
    if (it == m_byGuid.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<AssetMetadata> AssetDatabase::FindByVirtualPath(const std::string& virtualPath) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto pathIt = m_guidByPath.find(virtualPath);
    if (pathIt == m_guidByPath.end()) {
        return std::nullopt;
    }

    auto guidIt = m_byGuid.find(pathIt->second);
    if (guidIt == m_byGuid.end()) {
        return std::nullopt;
    }

    return guidIt->second;
}

std::optional<std::string> AssetDatabase::ResolveGuidByVirtualPath(const std::string& virtualPath) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_guidByPath.find(virtualPath);
    if (it == m_guidByPath.end()) {
        return std::nullopt;
    }
    return it->second;
}

} // namespace Resource
} // namespace Runtime
