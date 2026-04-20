#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace Runtime {
namespace Resource {

enum class ResourceState {
	Unloaded,
	Loading,
	Loaded,
	Failed,
};

class Resource {
public:
	Resource() = default;
	virtual ~Resource() = default;

	std::string Guid() const;
	std::string Type() const;
	std::string SourcePath() const;
	size_t SizeInBytes() const;
	ResourceState State() const;

	void SetGuid(std::string guid);
	void SetType(std::string type);
	void SetSourcePath(std::string sourcePath);
	void SetSizeInBytes(size_t bytes);
	void SetState(ResourceState state);

protected:
	mutable std::mutex m_mutex;
	std::string m_guid;
	std::string m_type;
	std::string m_sourcePath;
	size_t m_sizeInBytes = 0;
	ResourceState m_state = ResourceState::Unloaded;
};

class BinaryResource final : public Resource {
public:
	const std::vector<uint8_t>& Data() const;
	std::vector<uint8_t>& Data();
	void SetData(std::vector<uint8_t> data);

private:
	std::vector<uint8_t> m_data;
};

class TextResource final : public Resource {
public:
	const std::string& Text() const;
	void SetText(std::string text);

private:
	std::string m_text;
};

} // namespace Resource
} // namespace Runtime
