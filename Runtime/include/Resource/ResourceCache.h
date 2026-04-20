#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace Runtime {
namespace Resource {

class Resource;

class ResourceCache {
public:
	explicit ResourceCache(size_t maxEntries = 256);

	bool Put(const std::string& guid, std::shared_ptr<Resource> resource);
	std::shared_ptr<Resource> Get(const std::string& guid);
	std::shared_ptr<const Resource> Get(const std::string& guid) const;
	bool Erase(const std::string& guid);
	bool Contains(const std::string& guid) const;
	void Clear();

	void SetCapacity(size_t maxEntries);
	size_t Capacity() const;
	size_t Size() const;

private:
	struct Entry {
		std::shared_ptr<Resource> resource;
		uint64_t lastAccessTick = 0;
	};

	void EvictIfNeededLocked();

	mutable std::mutex m_mutex;
	size_t m_capacity = 256;
	uint64_t m_accessTick = 0;
	std::unordered_map<std::string, Entry> m_entries;
};

} // namespace Resource
} // namespace Runtime
