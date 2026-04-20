#include "Graphics/TextureManager.h"

#include <utility>

namespace Runtime {
namespace Graphics {

namespace {

bool ShouldUseSrgb(TextureManager::TextureUsage usage) {
    return usage == TextureManager::TextureUsage::Color;
}

} // namespace

void TextureManager::SetResolvers(void* context, ResolveBinaryResourceSyncFn syncResolver, ResolveBinaryResourceAsyncFn asyncResolver) {
    m_resolverContext = context;
    m_syncResolver = syncResolver;
    m_asyncResolver = asyncResolver;
}

std::shared_ptr<Texture> TextureManager::LoadSync(const TextureLoadRequest& request) {
    if (request.handle.IsValid()) {
        auto it = m_cache.find(request.handle);
        if (it != m_cache.end()) {
            return it->second;
        }
    }

    const uint8_t* byteData = request.bytes;
    size_t byteLength = request.byteLength;
    std::optional<std::vector<uint8_t>> resolvedBytes;
    if (!byteData || byteLength == 0) {
        if (!request.handle.IsValid() || !m_syncResolver) {
            return nullptr;
        }
        resolvedBytes = m_syncResolver(m_resolverContext, request.handle);
        if (!resolvedBytes.has_value() || resolvedBytes->empty()) {
            return nullptr;
        }
        byteData = resolvedBytes->data();
        byteLength = resolvedBytes->size();
    }

    auto texture = std::make_shared<Texture>();
    if (!texture->LoadFromMemory(byteData, byteLength, request.flipVertically, ShouldUseSrgb(request.usage))) {
        return nullptr;
    }

    if (request.handle.IsValid()) {
        m_cache[request.handle] = texture;
    }
    return texture;
}

std::future<std::optional<std::vector<uint8_t>>> TextureManager::ReadBytesAsync(TextureHandle handle) {
    if (!m_asyncResolver) {
        std::promise<std::optional<std::vector<uint8_t>>> rejected;
        rejected.set_value(std::nullopt);
        return rejected.get_future();
    }
    return m_asyncResolver(m_resolverContext, handle);
}

std::shared_ptr<Texture> TextureManager::Get(TextureHandle handle) const {
    auto it = m_cache.find(handle);
    if (it == m_cache.end()) {
        return nullptr;
    }
    return it->second;
}

bool TextureManager::Release(TextureHandle handle) {
    return m_cache.erase(handle) > 0;
}

void TextureManager::Clear() {
    m_cache.clear();
}

} // namespace Graphics
} // namespace Runtime
