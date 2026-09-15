#pragma once

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "AssetDatabase.h"
#include "AssetMetadata.h"
#include "Common/Handle.h"
#include "FileSystem.h"
#include "ResourceCache.h"

namespace Runtime {
namespace Resource {

using AssetHandle = ::Runtime::Handle;

class Resource;
class ResourceLoader;

template <typename T, uint32_t CapacityLog2 = 16>
class LockFreeMpmcQueue {
public:
	static constexpr size_t kCacheLineSize = 64;

	LockFreeMpmcQueue() {
		m_buffer = std::make_unique<Slot[]>(kCapacity);
		for (uint32_t i = 0; i < kCapacity; ++i) {
			m_buffer[i].sequence.store(i, std::memory_order_relaxed);
		}
	}

	bool TryEnqueue(T&& value) {
		uint32_t pos = m_enqueuePos.load(std::memory_order_relaxed);
		for (;;) {
			Slot& slot = m_buffer[pos & kMask];
			const uint32_t seq = slot.sequence.load(std::memory_order_acquire);
			const int32_t diff = static_cast<int32_t>(seq - pos);

			if (diff == 0) {
				if (m_enqueuePos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
					slot.value = std::move(value);
					slot.sequence.store(pos + 1, std::memory_order_release);
					m_dataEpoch.fetch_add(1, std::memory_order_release);
					m_dataEpoch.notify_one();
					return true;
				}
			} else if (diff < 0) {
				return false;
			} else {
				pos = m_enqueuePos.load(std::memory_order_relaxed);
			}
		}
	}

	bool TryDequeue(T& value) {
		uint32_t pos = m_dequeuePos.load(std::memory_order_relaxed);
		for (;;) {
			Slot& slot = m_buffer[pos & kMask];
			const uint32_t seq = slot.sequence.load(std::memory_order_acquire);
			const int32_t diff = static_cast<int32_t>(seq - (pos + 1));

			if (diff == 0) {
				if (m_dequeuePos.compare_exchange_weak(pos, pos + 1, std::memory_order_relaxed)) {
					value = std::move(slot.value);
					slot.sequence.store(pos + kCapacity, std::memory_order_release);
					m_spaceEpoch.fetch_add(1, std::memory_order_release);
					m_spaceEpoch.notify_one();
					return true;
				}
			} else if (diff < 0) {
				return false;
			} else {
				pos = m_dequeuePos.load(std::memory_order_relaxed);
			}
		}
	}

	bool WaitEnqueue(T&& value, const std::atomic<bool>& stopping) {
		while (!stopping.load(std::memory_order_acquire)) {
			if (TryEnqueue(std::move(value))) {
				return true;
			}

			const uint32_t observedEpoch = m_spaceEpoch.load(std::memory_order_acquire);
			if (TryEnqueue(std::move(value))) {
				return true;
			}
			if (stopping.load(std::memory_order_acquire)) {
				break;
			}
			m_spaceEpoch.wait(observedEpoch, std::memory_order_acquire);
		}
		return false;
	}

	bool WaitDequeue(T& value, const std::atomic<bool>& stopping) {
		while (!stopping.load(std::memory_order_acquire)) {
			if (TryDequeue(value)) {
				return true;
			}

			const uint32_t observedEpoch = m_dataEpoch.load(std::memory_order_acquire);
			if (TryDequeue(value)) {
				return true;
			}
			if (stopping.load(std::memory_order_acquire)) {
				break;
			}
			m_dataEpoch.wait(observedEpoch, std::memory_order_acquire);
		}
		return false;
	}

	void NotifyAllWaiters() {
		m_dataEpoch.fetch_add(1, std::memory_order_release);
		m_spaceEpoch.fetch_add(1, std::memory_order_release);
		m_dataEpoch.notify_all();
		m_spaceEpoch.notify_all();
	}

private:
	static_assert(CapacityLog2 == 8 || CapacityLog2 == 16, "CapacityLog2 must be 8 or 16.");
	static_assert(sizeof(std::atomic<uint32_t>) <= kCacheLineSize,
		"atomic<uint32_t> is larger than the configured cache line size");
	static constexpr uint32_t kCapacity = (1u << CapacityLog2);
	static constexpr uint32_t kMask = (kCapacity - 1u);

	struct Slot {
		std::atomic<uint32_t> sequence{0};
		T value{};
	};

	std::unique_ptr<Slot[]> m_buffer;
	alignas(kCacheLineSize) std::atomic<uint32_t> m_enqueuePos{0};
	char m_enqueuePad[kCacheLineSize - sizeof(std::atomic<uint32_t>)]{};
	alignas(kCacheLineSize) std::atomic<uint32_t> m_dequeuePos{0};
	char m_dequeuePad[kCacheLineSize - sizeof(std::atomic<uint32_t>)]{};
	alignas(kCacheLineSize) std::atomic<uint32_t> m_dataEpoch{0};
	alignas(kCacheLineSize) std::atomic<uint32_t> m_spaceEpoch{0};
};

class ResourceManager {
public:
	ResourceManager(size_t workerCount = 0, size_t cacheCapacity = 256);
	~ResourceManager();

	ResourceManager(const ResourceManager&) = delete;
	ResourceManager& operator=(const ResourceManager&) = delete;

	bool RegisterLoader(std::shared_ptr<ResourceLoader> loader);
	bool RegisterAsset(const AssetMetadata& metadata);
	bool RegisterHandle(AssetHandle handle, std::string guidOrVirtualPath);

	std::shared_ptr<Resource> LoadSync(const std::string& guidOrVirtualPath);
	std::shared_ptr<Resource> LoadSync(AssetHandle handle);
	std::future<std::shared_ptr<Resource>> LoadAsync(const std::string& guidOrVirtualPath);
	std::future<std::shared_ptr<Resource>> LoadAsync(AssetHandle handle);

	std::optional<std::string> ReadTextSync(const std::string& guidOrVirtualPath);
	std::optional<std::string> ReadTextSync(AssetHandle handle);
	std::future<std::optional<std::string>> ReadTextAsync(const std::string& guidOrVirtualPath);
	std::future<std::optional<std::string>> ReadTextAsync(AssetHandle handle);

	std::optional<std::vector<uint8_t>> ReadBinarySync(const std::string& guidOrVirtualPath);
	std::optional<std::vector<uint8_t>> ReadBinarySync(AssetHandle handle);
	std::future<std::optional<std::vector<uint8_t>>> ReadBinaryAsync(const std::string& guidOrVirtualPath);
	std::future<std::optional<std::vector<uint8_t>>> ReadBinaryAsync(AssetHandle handle);

	bool WriteTextSync(const std::string& virtualPath, const std::string& content);
	bool WriteBinarySync(const std::string& virtualPath, const std::vector<uint8_t>& content);
	std::future<bool> WriteTextAsync(const std::string& virtualPath, std::string content);
	std::future<bool> WriteBinaryAsync(const std::string& virtualPath, std::vector<uint8_t> content);

	std::shared_ptr<Resource> Get(const std::string& guidOrVirtualPath);
	bool Release(const std::string& guidOrVirtualPath);

	FileSystem& GetFileSystem();
	const FileSystem& GetFileSystem() const;

	AssetDatabase& GetAssetDatabase();
	const AssetDatabase& GetAssetDatabase() const;

private:
	std::optional<std::string> ResolveFromHandle(AssetHandle handle) const;

	struct AsyncResult {
		std::promise<std::shared_ptr<Resource>> promise;
	};

	struct IoTask {
		std::string guidOrVirtualPath;
		std::shared_ptr<AsyncResult> result;
	};

	struct DecodeTask {
		AssetMetadata metadata;
		std::shared_ptr<ResourceLoader> loader;
		std::vector<uint8_t> rawData;
		std::shared_ptr<AsyncResult> result;
	};

	struct UploadTask {
		AssetMetadata metadata;
		std::shared_ptr<ResourceLoader> loader;
		std::shared_ptr<Resource> resource;
		std::shared_ptr<AsyncResult> result;
	};

	std::optional<AssetMetadata> ResolveMetadata(const std::string& guidOrVirtualPath) const;
	std::shared_ptr<Resource> LoadFromMetadata(const AssetMetadata& metadata);
	std::future<std::shared_ptr<Resource>> EnqueueLoadTask(const std::string& guidOrVirtualPath);
	void FinalizeAndCache(const AssetMetadata& metadata, const std::shared_ptr<Resource>& resource);
	std::shared_ptr<ResourceLoader> FindLoaderForType(const std::string& type) const;
	void ResolveAndFail(std::shared_ptr<AsyncResult> result) const;
	void ResolveAndComplete(std::shared_ptr<AsyncResult> result, std::shared_ptr<Resource> value) const;

	void StartWorkers(size_t workerCount);
	void StopWorkers();
	void IoWorkerMain();
	void DecodeWorkerMain();
	void UploadWorkerMain();

	std::string DetectTypeByPath(const std::string& virtualPath) const;

	mutable std::mutex m_mutex;
	FileSystem m_fileSystem;
	AssetDatabase m_assetDatabase;
	ResourceCache m_cache;
	std::unordered_map<AssetHandle, std::string> m_handleToPath;
	std::unordered_map<std::string, std::shared_ptr<ResourceLoader>> m_loaders;

	std::atomic<bool> m_stopping{false};
	LockFreeMpmcQueue<IoTask> m_ioTasks;
	LockFreeMpmcQueue<DecodeTask> m_decodeTasks;
	LockFreeMpmcQueue<UploadTask> m_uploadTasks;

	std::vector<std::thread> m_ioWorkers;
	std::vector<std::thread> m_decodeWorkers;
	std::thread m_uploadWorker;
};

} // namespace Resource
} // namespace Runtime
