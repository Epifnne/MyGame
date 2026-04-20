#include "Resource/Resource.h"

#include <utility>

namespace Runtime {
namespace Resource {

std::string Resource::Guid() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_guid;
}

std::string Resource::Type() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_type;
}

std::string Resource::SourcePath() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_sourcePath;
}

size_t Resource::SizeInBytes() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_sizeInBytes;
}

ResourceState Resource::State() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_state;
}

void Resource::SetGuid(std::string guid) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_guid = std::move(guid);
}

void Resource::SetType(std::string type) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_type = std::move(type);
}

void Resource::SetSourcePath(std::string sourcePath) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_sourcePath = std::move(sourcePath);
}

void Resource::SetSizeInBytes(size_t bytes) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_sizeInBytes = bytes;
}

void Resource::SetState(ResourceState state) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_state = state;
}

const std::vector<uint8_t>& BinaryResource::Data() const {
    return m_data;
}

std::vector<uint8_t>& BinaryResource::Data() {
    return m_data;
}

void BinaryResource::SetData(std::vector<uint8_t> data) {
    m_data = std::move(data);
    SetSizeInBytes(m_data.size());
}

const std::string& TextResource::Text() const {
    return m_text;
}

void TextResource::SetText(std::string text) {
    m_text = std::move(text);
    SetSizeInBytes(m_text.size());
}

} // namespace Resource
} // namespace Runtime
