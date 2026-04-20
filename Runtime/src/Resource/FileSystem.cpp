#include "Resource/FileSystem.h"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace Runtime {
namespace Resource {
namespace {

std::string Normalize(const std::string& path) {
    std::string value = path;
    for (char& c : value) {
        if (c == '\\') {
            c = '/';
        }
    }
    return value;
}

} // namespace

bool FileSystem::Mount(const std::string& mountName, const std::string& absoluteOrRelativeRoot) {
    if (mountName.empty() || absoluteOrRelativeRoot.empty()) {
        return false;
    }

    std::filesystem::path rootPath = std::filesystem::weakly_canonical(absoluteOrRelativeRoot);
    std::lock_guard<std::shared_mutex> lock(m_mutex);
    m_mounts[mountName] = Normalize(rootPath.generic_string());
    return true;
}

bool FileSystem::Unmount(const std::string& mountName) {
    std::lock_guard<std::shared_mutex> lock(m_mutex);
    return m_mounts.erase(mountName) > 0;
}

std::string FileSystem::ResolvePath(const std::string& virtualPath) const {
    if (virtualPath.empty()) {
        return {};
    }

    const size_t splitPos = virtualPath.find(":/");
    if (splitPos == std::string::npos) {
        return Normalize(virtualPath);
    }

    const std::string mountName = virtualPath.substr(0, splitPos);
    const std::string relativePart = virtualPath.substr(splitPos + 2);

    std::shared_lock<std::shared_mutex> lock(m_mutex);
    auto it = m_mounts.find(mountName);
    if (it == m_mounts.end()) {
        return Normalize(virtualPath);
    }

    std::filesystem::path finalPath = std::filesystem::path(it->second) / std::filesystem::path(relativePart);
    return Normalize(finalPath.generic_string());
}

bool FileSystem::Exists(const std::string& virtualPath) const {
    const std::string resolved = ResolvePath(virtualPath);
    if (resolved.empty()) {
        return false;
    }
    return std::filesystem::exists(std::filesystem::path(resolved));
}

std::optional<std::string> FileSystem::ReadTextFile(const std::string& virtualPath) const {
    const std::string resolved = ResolvePath(virtualPath);
    std::ifstream input(resolved, std::ios::in);
    if (!input.is_open()) {
        return std::nullopt;
    }

    std::ostringstream stream;
    stream << input.rdbuf();
    return stream.str();
}

std::optional<std::vector<uint8_t>> FileSystem::ReadBinaryFile(const std::string& virtualPath) const {
    const std::string resolved = ResolvePath(virtualPath);
    std::ifstream input(resolved, std::ios::binary);
    if (!input.is_open()) {
        return std::nullopt;
    }

    input.seekg(0, std::ios::end);
    const std::streamoff size = input.tellg();
    if (size < 0) {
        return std::nullopt;
    }
    input.seekg(0, std::ios::beg);

    std::vector<uint8_t> data(static_cast<size_t>(size));
    if (size > 0) {
        input.read(reinterpret_cast<char*>(data.data()), size);
    }
    return data;
}

bool FileSystem::WriteTextFile(const std::string& virtualPath, const std::string& content) {
    const std::string resolved = ResolvePath(virtualPath);
    if (resolved.empty()) {
        return false;
    }

    const std::filesystem::path path(resolved);
    const std::filesystem::path parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
    }

    std::ofstream output(resolved, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    output << content;
    return static_cast<bool>(output);
}

bool FileSystem::WriteBinaryFile(const std::string& virtualPath, const std::vector<uint8_t>& content) {
    const std::string resolved = ResolvePath(virtualPath);
    if (resolved.empty()) {
        return false;
    }

    const std::filesystem::path path(resolved);
    const std::filesystem::path parent = path.parent_path();
    if (!parent.empty()) {
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
    }

    std::ofstream output(resolved, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        return false;
    }

    if (!content.empty()) {
        output.write(reinterpret_cast<const char*>(content.data()), static_cast<std::streamsize>(content.size()));
    }
    return static_cast<bool>(output);
}

} // namespace Resource
} // namespace Runtime
