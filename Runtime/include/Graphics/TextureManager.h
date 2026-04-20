#pragma once

#include <future>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "Graphics/Texture.h"
#include "Common/Handle.h"

namespace Runtime {
namespace Graphics {

using TextureHandle = ::Runtime::Handle;

class TextureManager {
public:
    using ResolveBinaryResourceSyncFn = std::optional<std::vector<uint8_t>>(*)(void* context, TextureHandle);
    using ResolveBinaryResourceAsyncFn = std::future<std::optional<std::vector<uint8_t>>>(*)(void* context, TextureHandle);

    enum class TextureUsage {
        Color,
        Normal,
        ORM,
        Data
    };

    struct TextureLoadRequest {
        TextureHandle handle;
        const uint8_t* bytes = nullptr;
        size_t byteLength = 0;
        bool flipVertically = true;
        TextureUsage usage = TextureUsage::Color;
    };

    void SetResolvers(void* context, ResolveBinaryResourceSyncFn syncResolver, ResolveBinaryResourceAsyncFn asyncResolver);

    std::shared_ptr<Texture> LoadSync(const TextureLoadRequest& request);
    std::future<std::optional<std::vector<uint8_t>>> ReadBytesAsync(TextureHandle handle);

    std::shared_ptr<Texture> Get(TextureHandle handle) const;
    bool Release(TextureHandle handle);
    void Clear();

private:
    void* m_resolverContext = nullptr;
    ResolveBinaryResourceSyncFn m_syncResolver = nullptr;
    ResolveBinaryResourceAsyncFn m_asyncResolver = nullptr;
    std::unordered_map<TextureHandle, std::shared_ptr<Texture>> m_cache;
};

} // namespace Graphics
} // namespace Runtime
