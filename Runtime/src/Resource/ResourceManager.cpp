#include "Resource/ResourceManager.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <filesystem>
#include <utility>

#include "Resource/MaterialLoader.h"
#include "Resource/MeshLoader.h"
#include "Resource/Resource.h"
#include "Resource/ResourceLoader.h"
#include "Resource/ShaderLoader.h"
#include "Resource/TextureLoader.h"

namespace Runtime {
namespace Resource {

ResourceManager::ResourceManager(size_t workerCount, size_t cacheCapacity)
    : m_cache(cacheCapacity) {
    RegisterLoader(std::make_shared<TextureLoader>());
    RegisterLoader(std::make_shared<MeshLoader>());
    RegisterLoader(std::make_shared<MaterialLoader>());
    RegisterLoader(std::make_shared<ShaderLoader>());

    const size_t count = workerCount == 0 ? std::max<size_t>(1, std::thread::hardware_concurrency()) : workerCount;
    StartWorkers(count);
}

ResourceManager::~ResourceManager() {
    StopWorkers();
}

bool ResourceManager::RegisterLoader(std::shared_ptr<ResourceLoader> loader) {
    if (!loader) {
        return false;
    }

    const std::string type = loader->ResourceType();
    if (type.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_loaders[type] = std::move(loader);
    return true;
}

bool ResourceManager::RegisterAsset(const AssetMetadata& metadata) {
    if (metadata.guid.empty() || metadata.virtualPath.empty()) {
        return false;
    }
    return m_assetDatabase.Register(metadata);
}

bool ResourceManager::RegisterHandle(AssetHandle handle, std::string guidOrVirtualPath) {
    if (!handle.IsValid() || guidOrVirtualPath.empty()) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_handleToPath.find(handle);
    if (it != m_handleToPath.end() && it->second != guidOrVirtualPath) {
        return false;
    }
    m_handleToPath[handle] = std::move(guidOrVirtualPath);
    return true;
}

std::optional<std::string> ResourceManager::ResolveFromHandle(AssetHandle handle) const {
    if (!handle.IsValid()) {
        return std::nullopt;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_handleToPath.find(handle);
    if (it == m_handleToPath.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::shared_ptr<Resource> ResourceManager::LoadSync(const std::string& guidOrVirtualPath) {
    const auto metadata = ResolveMetadata(guidOrVirtualPath);
    if (!metadata.has_value()) {
        return nullptr;
    }
    return LoadFromMetadata(metadata.value());
}

std::shared_ptr<Resource> ResourceManager::LoadSync(AssetHandle handle) {
    const auto key = ResolveFromHandle(handle);
    if (!key.has_value()) {
        return nullptr;
    }
    return LoadSync(key.value());
}

std::future<std::shared_ptr<Resource>> ResourceManager::LoadAsync(const std::string& guidOrVirtualPath) {
    return EnqueueLoadTask(guidOrVirtualPath);
}

std::future<std::shared_ptr<Resource>> ResourceManager::LoadAsync(AssetHandle handle) {
    const auto key = ResolveFromHandle(handle);
    if (!key.has_value()) {
        std::promise<std::shared_ptr<Resource>> rejected;
        rejected.set_value(nullptr);
        return rejected.get_future();
    }
    return LoadAsync(key.value());
}

std::optional<std::string> ResourceManager::ReadTextSync(const std::string& guidOrVirtualPath) {
    std::shared_ptr<Resource> resource = LoadSync(guidOrVirtualPath);
    auto textResource = std::dynamic_pointer_cast<TextResource>(resource);
    if (!textResource) {
        return std::nullopt;
    }
    return textResource->Text();
}

std::optional<std::string> ResourceManager::ReadTextSync(AssetHandle handle) {
    const auto key = ResolveFromHandle(handle);
    if (!key.has_value()) {
        return std::nullopt;
    }
    return ReadTextSync(key.value());
}

std::future<std::optional<std::string>> ResourceManager::ReadTextAsync(const std::string& guidOrVirtualPath) {
    auto fut = LoadAsync(guidOrVirtualPath);
    return std::async(std::launch::async, [f = std::move(fut)]() mutable {
        auto resource = f.get();
        auto textResource = std::dynamic_pointer_cast<TextResource>(resource);
        if (!textResource) {
            return std::optional<std::string>{};
        }
        return std::optional<std::string>{textResource->Text()};
    });
}

std::future<std::optional<std::string>> ResourceManager::ReadTextAsync(AssetHandle handle) {
    const auto key = ResolveFromHandle(handle);
    if (!key.has_value()) {
        std::promise<std::optional<std::string>> rejected;
        rejected.set_value(std::nullopt);
        return rejected.get_future();
    }
    return ReadTextAsync(key.value());
}

std::optional<std::vector<uint8_t>> ResourceManager::ReadBinarySync(const std::string& guidOrVirtualPath) {
    std::shared_ptr<Resource> resource = LoadSync(guidOrVirtualPath);
    auto binaryResource = std::dynamic_pointer_cast<BinaryResource>(resource);
    if (!binaryResource) {
        return std::nullopt;
    }
    return binaryResource->Data();
}

std::optional<std::vector<uint8_t>> ResourceManager::ReadBinarySync(AssetHandle handle) {
    const auto key = ResolveFromHandle(handle);
    if (!key.has_value()) {
        return std::nullopt;
    }
    return ReadBinarySync(key.value());
}

std::future<std::optional<std::vector<uint8_t>>> ResourceManager::ReadBinaryAsync(const std::string& guidOrVirtualPath) {
    auto fut = LoadAsync(guidOrVirtualPath);
    return std::async(std::launch::async, [f = std::move(fut)]() mutable {
        auto resource = f.get();
        auto binaryResource = std::dynamic_pointer_cast<BinaryResource>(resource);
        if (!binaryResource) {
            return std::optional<std::vector<uint8_t>>{};
        }
        return std::optional<std::vector<uint8_t>>{binaryResource->Data()};
    });
}

std::future<std::optional<std::vector<uint8_t>>> ResourceManager::ReadBinaryAsync(AssetHandle handle) {
    const auto key = ResolveFromHandle(handle);
    if (!key.has_value()) {
        std::promise<std::optional<std::vector<uint8_t>>> rejected;
        rejected.set_value(std::nullopt);
        return rejected.get_future();
    }
    return ReadBinaryAsync(key.value());
}

bool ResourceManager::WriteTextSync(const std::string& virtualPath, const std::string& content) {
    return m_fileSystem.WriteTextFile(virtualPath, content);
}

bool ResourceManager::WriteBinarySync(const std::string& virtualPath, const std::vector<uint8_t>& content) {
    return m_fileSystem.WriteBinaryFile(virtualPath, content);
}

std::future<bool> ResourceManager::WriteTextAsync(const std::string& virtualPath, std::string content) {
    return std::async(std::launch::async, [this, virtualPath, content = std::move(content)]() {
        return WriteTextSync(virtualPath, content);
    });
}

std::future<bool> ResourceManager::WriteBinaryAsync(const std::string& virtualPath, std::vector<uint8_t> content) {
    return std::async(std::launch::async, [this, virtualPath, content = std::move(content)]() {
        return WriteBinarySync(virtualPath, content);
    });
}

std::shared_ptr<Resource> ResourceManager::Get(const std::string& guidOrVirtualPath) {
    const auto metadata = ResolveMetadata(guidOrVirtualPath);
    if (!metadata.has_value()) {
        return nullptr;
    }
    return m_cache.Get(metadata->guid);
}

bool ResourceManager::Release(const std::string& guidOrVirtualPath) {
    const auto metadata = ResolveMetadata(guidOrVirtualPath);
    if (!metadata.has_value()) {
        return false;
    }
    return m_cache.Erase(metadata->guid);
}

FileSystem& ResourceManager::GetFileSystem() {
    return m_fileSystem;
}

const FileSystem& ResourceManager::GetFileSystem() const {
    return m_fileSystem;
}

AssetDatabase& ResourceManager::GetAssetDatabase() {
    return m_assetDatabase;
}

const AssetDatabase& ResourceManager::GetAssetDatabase() const {
    return m_assetDatabase;
}

std::optional<AssetMetadata> ResourceManager::ResolveMetadata(const std::string& guidOrVirtualPath) const {
    if (guidOrVirtualPath.empty()) {
        return std::nullopt;
    }

    if (auto byGuid = m_assetDatabase.FindByGuid(guidOrVirtualPath); byGuid.has_value()) {
        return byGuid;
    }

    if (auto byPath = m_assetDatabase.FindByVirtualPath(guidOrVirtualPath); byPath.has_value()) {
        return byPath;
    }

    AssetMetadata metadata;
    metadata.guid = guidOrVirtualPath;
    metadata.virtualPath = guidOrVirtualPath;
    metadata.sourcePath = guidOrVirtualPath;
    metadata.type = DetectTypeByPath(guidOrVirtualPath);
    return metadata;
}

std::shared_ptr<Resource> ResourceManager::LoadFromMetadata(const AssetMetadata& metadata) {
    if (auto cached = m_cache.Get(metadata.guid)) {
        return cached;
    }

    auto loader = FindLoaderForType(metadata.type);
    if (!loader) {
        return nullptr;
    }

    auto rawData = loader->Read(metadata, m_fileSystem);
    if (!rawData.has_value()) {
        return nullptr;
    }

    auto resource = loader->Decode(metadata, std::move(rawData.value()));
    if (!resource) {
        return nullptr;
    }

    if (!loader->Upload(metadata, resource)) {
        return nullptr;
    }

    FinalizeAndCache(metadata, resource);
    return resource;
}

void ResourceManager::FinalizeAndCache(const AssetMetadata& metadata, const std::shared_ptr<Resource>& resource) {
    if (!resource) {
        return;
    }

    resource->SetGuid(metadata.guid);
    if (resource->Type().empty()) {
        resource->SetType(metadata.type);
    }
    if (resource->SourcePath().empty()) {
        resource->SetSourcePath(metadata.sourcePath.empty() ? metadata.virtualPath : metadata.sourcePath);
    }

    if (resource->State() != ResourceState::Loaded) {
        resource->SetState(ResourceState::Loaded);
    }

    m_cache.Put(metadata.guid, resource);
}

std::shared_ptr<ResourceLoader> ResourceManager::FindLoaderForType(const std::string& type) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_loaders.find(type);
    if (it == m_loaders.end()) {
        return nullptr;
    }
    return it->second;
}

void ResourceManager::ResolveAndFail(std::shared_ptr<AsyncResult> result) const {
    ResolveAndComplete(std::move(result), nullptr);
}

void ResourceManager::ResolveAndComplete(std::shared_ptr<AsyncResult> result, std::shared_ptr<Resource> value) const {
    if (!result) {
        return;
    }

    try {
        result->promise.set_value(std::move(value));
    } catch (const std::future_error&) {
        // Promise may already be resolved.
    }
}

std::future<std::shared_ptr<Resource>> ResourceManager::EnqueueLoadTask(const std::string& guidOrVirtualPath) {
    auto result = std::make_shared<AsyncResult>();
    auto future = result->promise.get_future();

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_stopping) {
            result->promise.set_value(nullptr);
            return future;
        }
        m_ioTasks.push_back(IoTask{guidOrVirtualPath, result});
    }

    m_ioCv.notify_one();
    return future;
}

void ResourceManager::StartWorkers(size_t workerCount) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_ioWorkers.empty() || !m_decodeWorkers.empty() || m_uploadWorker.joinable()) {
        return;
    }

    const size_t ioWorkerCount = std::max<size_t>(1, std::min<size_t>(2, workerCount));
    const size_t decodeWorkerCount = std::max<size_t>(1, workerCount);

    m_ioWorkers.reserve(ioWorkerCount);
    for (size_t i = 0; i < ioWorkerCount; ++i) {
        m_ioWorkers.emplace_back([this]() { IoWorkerMain(); });
    }

    m_decodeWorkers.reserve(decodeWorkerCount);
    for (size_t i = 0; i < decodeWorkerCount; ++i) {
        m_decodeWorkers.emplace_back([this]() { DecodeWorkerMain(); });
    }

    m_uploadWorker = std::thread([this]() { UploadWorkerMain(); });
}

void ResourceManager::StopWorkers() {
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_stopping = true;
    }
    m_ioCv.notify_all();
    m_decodeCv.notify_all();
    m_uploadCv.notify_all();

    for (std::thread& worker : m_ioWorkers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    m_ioWorkers.clear();

    for (std::thread& worker : m_decodeWorkers) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    m_decodeWorkers.clear();

    if (m_uploadWorker.joinable()) {
        m_uploadWorker.join();
    }

    std::deque<IoTask> pendingIo;
    std::deque<DecodeTask> pendingDecode;
    std::deque<UploadTask> pendingUpload;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        pendingIo.swap(m_ioTasks);
        pendingDecode.swap(m_decodeTasks);
        pendingUpload.swap(m_uploadTasks);
    }

    for (auto& task : pendingIo) {
        ResolveAndFail(task.result);
    }
    for (auto& task : pendingDecode) {
        ResolveAndFail(task.result);
    }
    for (auto& task : pendingUpload) {
        ResolveAndFail(task.result);
    }
}

void ResourceManager::IoWorkerMain() {
    while (true) {
        IoTask task;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_ioCv.wait(lock, [this]() {
                return m_stopping || !m_ioTasks.empty();
            });

            if (m_stopping && m_ioTasks.empty()) {
                return;
            }

            task = std::move(m_ioTasks.front());
            m_ioTasks.pop_front();
        }

        const auto metadata = ResolveMetadata(task.guidOrVirtualPath);
        if (!metadata.has_value()) {
            ResolveAndFail(task.result);
            continue;
        }

        if (auto cached = m_cache.Get(metadata->guid)) {
            ResolveAndComplete(task.result, cached);
            continue;
        }

        auto loader = FindLoaderForType(metadata->type);
        if (!loader) {
            ResolveAndFail(task.result);
            continue;
        }

        auto rawData = loader->Read(metadata.value(), m_fileSystem);
        if (!rawData.has_value()) {
            ResolveAndFail(task.result);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_decodeTasks.push_back(DecodeTask{metadata.value(), loader, std::move(rawData.value()), task.result});
        }
        m_decodeCv.notify_one();
    }
}

void ResourceManager::DecodeWorkerMain() {
    while (true) {
        DecodeTask task;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_decodeCv.wait(lock, [this]() {
                return m_stopping || !m_decodeTasks.empty();
            });

            if (m_stopping && m_decodeTasks.empty()) {
                return;
            }

            task = std::move(m_decodeTasks.front());
            m_decodeTasks.pop_front();
        }

        auto resource = task.loader->Decode(task.metadata, std::move(task.rawData));
        if (!resource) {
            ResolveAndFail(task.result);
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_uploadTasks.push_back(UploadTask{std::move(task.metadata), std::move(task.loader), std::move(resource), task.result});
        }
        m_uploadCv.notify_one();
    }
}

void ResourceManager::UploadWorkerMain() {
    while (true) {
        UploadTask task;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_uploadCv.wait(lock, [this]() {
                return m_stopping || !m_uploadTasks.empty();
            });

            if (m_stopping && m_uploadTasks.empty()) {
                return;
            }

            task = std::move(m_uploadTasks.front());
            m_uploadTasks.pop_front();
        }

        if (!task.loader->Upload(task.metadata, task.resource)) {
            ResolveAndFail(task.result);
            continue;
        }

        FinalizeAndCache(task.metadata, task.resource);
        ResolveAndComplete(task.result, task.resource);
    }
}

std::string ResourceManager::DetectTypeByPath(const std::string& virtualPath) const {
    std::filesystem::path path(virtualPath);
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });

    if (ext == ".vert" || ext == ".frag" || ext == ".glsl" || ext == ".shader") {
        return "shader";
    }
    if (ext == ".json" || ext == ".mat") {
        return "material";
    }
    if (ext == ".obj" || ext == ".fbx" || ext == ".mesh") {
        return "mesh";
    }
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".dds" || ext == ".bmp") {
        return "texture";
    }

    return "texture";
}

} // namespace Resource
} // namespace Runtime
