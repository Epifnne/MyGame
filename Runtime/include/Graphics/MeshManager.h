#pragma once

#include <future>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "Graphics/Mesh.h"
#include "Common/Handle.h"

namespace Runtime {
namespace Graphics {

using MeshHandle = ::Runtime::Handle;

class MeshManager {
public:
    using ResolveBinaryResourceSyncFn = std::optional<std::vector<uint8_t>>(*)(void* context, MeshHandle);
    using ResolveBinaryResourceAsyncFn = std::future<std::optional<std::vector<uint8_t>>>(*)(void* context, MeshHandle);

    void SetResolvers(void* context, ResolveBinaryResourceSyncFn syncResolver, ResolveBinaryResourceAsyncFn asyncResolver);

    std::shared_ptr<Mesh> CreateFromVertices(MeshHandle handle, const float* vertices, size_t vertexCount);
    std::shared_ptr<Mesh> LoadFromBinarySync(MeshHandle handle);
    std::future<std::optional<std::vector<uint8_t>>> ReadBytesAsync(MeshHandle handle);

    std::shared_ptr<Mesh> Get(MeshHandle handle) const;
    bool Release(MeshHandle handle);
    void Clear();

private:
    void* m_resolverContext = nullptr;
    ResolveBinaryResourceSyncFn m_syncResolver = nullptr;
    ResolveBinaryResourceAsyncFn m_asyncResolver = nullptr;
    std::unordered_map<MeshHandle, std::shared_ptr<Mesh>> m_cache;
};

} // namespace Graphics
} // namespace Runtime
