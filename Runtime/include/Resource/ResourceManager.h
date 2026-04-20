#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
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

	bool m_stopping = false;
	std::deque<IoTask> m_ioTasks;
	std::deque<DecodeTask> m_decodeTasks;
	std::deque<UploadTask> m_uploadTasks;

	std::vector<std::thread> m_ioWorkers;
	std::vector<std::thread> m_decodeWorkers;
	std::thread m_uploadWorker;

	std::condition_variable m_ioCv;
	std::condition_variable m_decodeCv;
	std::condition_variable m_uploadCv;
};

} // namespace Resource
} // namespace Runtime
