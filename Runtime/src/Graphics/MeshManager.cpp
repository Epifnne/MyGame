#include "Graphics/MeshManager.h"

#include <utility>

namespace Runtime {
namespace Graphics {

void MeshManager::SetResolvers(void* context, ResolveBinaryResourceSyncFn syncResolver, ResolveBinaryResourceAsyncFn asyncResolver) {
    m_resolverContext = context;
    m_syncResolver = syncResolver;
    m_asyncResolver = asyncResolver;
}

std::shared_ptr<Mesh> MeshManager::CreateFromVertices(MeshHandle handle, const float* vertices, size_t vertexCount) {
    if (!handle.IsValid() || !vertices || vertexCount == 0) {
        return nullptr;
    }

    auto mesh = std::make_shared<Mesh>();
    if (!mesh->Create(vertices, vertexCount)) {
        return nullptr;
    }

    m_cache[handle] = mesh;
    return mesh;
}

std::shared_ptr<Mesh> MeshManager::LoadFromBinarySync(MeshHandle handle) {
    auto it = m_cache.find(handle);
    if (it != m_cache.end()) {
        return it->second;
    }

    if (!m_syncResolver) {
        return nullptr;
    }

    auto bytes = m_syncResolver(m_resolverContext, handle);
    if (!bytes.has_value() || bytes->empty()) {
        return nullptr;
    }

    if ((bytes->size() % sizeof(float)) != 0) {
        return nullptr;
    }

    const float* vertices = reinterpret_cast<const float*>(bytes->data());
    const size_t vertexCount = bytes->size() / sizeof(float);

    auto mesh = std::make_shared<Mesh>();
    if (!mesh->Create(vertices, vertexCount)) {
        return nullptr;
    }

    m_cache[handle] = mesh;
    return mesh;
}

std::future<std::optional<std::vector<uint8_t>>> MeshManager::ReadBytesAsync(MeshHandle handle) {
    if (!m_asyncResolver) {
        std::promise<std::optional<std::vector<uint8_t>>> rejected;
        rejected.set_value(std::nullopt);
        return rejected.get_future();
    }
    return m_asyncResolver(m_resolverContext, handle);
}

std::shared_ptr<Mesh> MeshManager::Get(MeshHandle handle) const {
    auto it = m_cache.find(handle);
    if (it == m_cache.end()) {
        return nullptr;
    }
    return it->second;
}

bool MeshManager::Release(MeshHandle handle) {
    return m_cache.erase(handle) > 0;
}

void MeshManager::Clear() {
    m_cache.clear();
}

} // namespace Graphics
} // namespace Runtime
