#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

#include "Resource/AssetMetadata.h"
#include "Resource/FileSystem.h"
#include "Resource/ImportPipeline.h"
#include "Resource/Resource.h"
#include "Resource/ResourceManager.h"

using Runtime::Resource::AssetMetadata;
using Runtime::Resource::FileSystem;
using Runtime::Resource::ImportPipeline;
using Runtime::Resource::ResourceManager;
using Runtime::Resource::ResourceState;
using Runtime::Resource::TextResource;

namespace {

class TempDir {
public:
    TempDir() {
        const auto base = std::filesystem::temp_directory_path();
        m_path = base / std::filesystem::path("mygame_resource_tests");
        std::error_code ec;
        std::filesystem::remove_all(m_path, ec);
        std::filesystem::create_directories(m_path, ec);
    }

    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(m_path, ec);
    }

    const std::filesystem::path& Path() const { return m_path; }

private:
    std::filesystem::path m_path;
};

} // namespace

TEST(ResourceFileSystemTest, MountResolveAndReadText) {
    TempDir dir;
    const auto source = dir.Path() / "hello.txt";

    {
        std::ofstream out(source.string(), std::ios::out | std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out << "resource-module";
    }

    FileSystem fs;
    ASSERT_TRUE(fs.Mount("assets", dir.Path().string()));

    const std::string resolved = fs.ResolvePath("assets:/hello.txt");
    EXPECT_FALSE(resolved.empty());
    EXPECT_TRUE(fs.Exists("assets:/hello.txt"));

    auto text = fs.ReadTextFile("assets:/hello.txt");
    ASSERT_TRUE(text.has_value());
    EXPECT_EQ(text.value(), "resource-module");
}

TEST(ResourceManagerTest, LoadSyncReadsRegisteredShaderAsset) {
    TempDir dir;
    const auto shaderPath = dir.Path() / "basic.vert";

    {
        std::ofstream out(shaderPath.string(), std::ios::out | std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out << "void main() {}";
    }

    ResourceManager manager(2, 16);

    AssetMetadata metadata;
    metadata.guid = ImportPipeline::BuildGuid(shaderPath.string(), "shader");
    metadata.type = "shader";
    metadata.sourcePath = shaderPath.string();
    metadata.virtualPath = "assets:/basic.vert";

    ASSERT_TRUE(manager.RegisterAsset(metadata));

    auto resource = manager.LoadSync(metadata.guid);
    ASSERT_NE(resource, nullptr);
    EXPECT_EQ(resource->State(), ResourceState::Loaded);
    EXPECT_EQ(resource->Type(), "shader");

    auto textResource = std::dynamic_pointer_cast<TextResource>(resource);
    ASSERT_NE(textResource, nullptr);
    EXPECT_EQ(textResource->Text(), "void main() {}");
}

TEST(ResourceManagerTest, LoadAsyncReturnsFutureAndCachesResult) {
    TempDir dir;
    const auto matPath = dir.Path() / "metal.mat";

    {
        std::ofstream out(matPath.string(), std::ios::out | std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out << "{\"name\":\"metal\"}";
    }

    ResourceManager manager(2, 16);

    AssetMetadata metadata;
    metadata.guid = ImportPipeline::BuildGuid(matPath.string(), "material");
    metadata.type = "material";
    metadata.sourcePath = matPath.string();
    metadata.virtualPath = "assets:/metal.mat";

    ASSERT_TRUE(manager.RegisterAsset(metadata));

    auto future = manager.LoadAsync(metadata.guid);
    auto fromFuture = future.get();
    ASSERT_NE(fromFuture, nullptr);
    EXPECT_EQ(fromFuture->State(), ResourceState::Loaded);

    auto fromCache = manager.Get(metadata.guid);
    ASSERT_NE(fromCache, nullptr);
    EXPECT_EQ(fromCache.get(), fromFuture.get());
}

TEST(ResourceManagerTest, UnifiedReadTextSupportsSyncAndAsyncIoModes) {
    TempDir dir;
    const auto shaderPath = dir.Path() / "unified.vert";

    {
        std::ofstream out(shaderPath.string(), std::ios::out | std::ios::trunc);
        ASSERT_TRUE(out.is_open());
        out << "void main(){gl_Position=vec4(1.0);}";
    }

    ResourceManager manager(2, 16);

    AssetMetadata metadata;
    metadata.guid = ImportPipeline::BuildGuid(shaderPath.string(), "shader");
    metadata.type = "shader";
    metadata.sourcePath = shaderPath.string();
    metadata.virtualPath = "assets:/unified.vert";
    ASSERT_TRUE(manager.RegisterAsset(metadata));

    auto syncText = manager.ReadTextSync(metadata.guid);
    ASSERT_TRUE(syncText.has_value());

    auto asyncText = manager.ReadTextAsync(metadata.guid).get();
    ASSERT_TRUE(asyncText.has_value());
    EXPECT_EQ(syncText.value(), asyncText.value());
}

TEST(ResourceManagerTest, WriteAndReadBinaryAreManagedByResourceModule) {
    TempDir dir;
    ResourceManager manager(2, 16);

    ASSERT_TRUE(manager.GetFileSystem().Mount("assets", dir.Path().string()));

    const std::vector<uint8_t> source = {1, 3, 5, 7, 9, 11};
    ASSERT_TRUE(manager.WriteBinarySync("assets:/payload.bin", source));

    auto syncBytes = manager.ReadBinarySync("assets:/payload.bin");
    ASSERT_TRUE(syncBytes.has_value());
    EXPECT_EQ(syncBytes.value(), source);

    auto asyncBytes = manager.ReadBinaryAsync("assets:/payload.bin").get();
    ASSERT_TRUE(asyncBytes.has_value());
    EXPECT_EQ(asyncBytes.value(), source);
}
