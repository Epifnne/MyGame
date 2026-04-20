#include "Resource/ResourceCache.h"

#include <limits>

namespace Runtime {
namespace Resource {

ResourceCache::ResourceCache(size_t maxEntries)
    : m_capacity(maxEntries == 0 ? 1 : maxEntries) {}

bool ResourceCache::Put(const std::string& guid, std::shared_ptr<Resource> resource) {
    if (guid.empty() || !resource) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    Entry entry;
    entry.resource = std::move(resource);
    entry.lastAccessTick = ++m_accessTick;
    m_entries[guid] = std::move(entry);
    EvictIfNeededLocked();
    return true;
}

std::shared_ptr<Resource> ResourceCache::Get(const std::string& guid) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_entries.find(guid);
    if (it == m_entries.end()) {
        return nullptr;
    }
    it->second.lastAccessTick = ++m_accessTick;
    return it->second.resource;
}

std::shared_ptr<const Resource> ResourceCache::Get(const std::string& guid) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_entries.find(guid);
    if (it == m_entries.end()) {
        return nullptr;
    }
    return it->second.resource;
}

bool ResourceCache::Erase(const std::string& guid) {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_entries.erase(guid) > 0;
}

bool ResourceCache::Contains(const std::string& guid) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_entries.find(guid) != m_entries.end();
}

void ResourceCache::Clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_entries.clear();
}

void ResourceCache::SetCapacity(size_t maxEntries) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_capacity = maxEntries == 0 ? 1 : maxEntries;
    EvictIfNeededLocked();
}

size_t ResourceCache::Capacity() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_capacity;
}

size_t ResourceCache::Size() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_entries.size();
}

void ResourceCache::EvictIfNeededLocked() {
    while (m_entries.size() > m_capacity) {
        auto victim = m_entries.end();
        uint64_t minTick = std::numeric_limits<uint64_t>::max();

        for (auto it = m_entries.begin(); it != m_entries.end(); ++it) {
            if (!it->second.resource || it->second.resource.use_count() <= 1) {
                if (it->second.lastAccessTick < minTick) {
                    minTick = it->second.lastAccessTick;
                    victim = it;
                }
            }
        }

        if (victim == m_entries.end()) {
            break;
        }

        m_entries.erase(victim);
    }
}

} // namespace Resource
} // namespace Runtime
