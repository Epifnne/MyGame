#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace Runtime {
namespace Network {

class PacketBuffer {
public:
	void Append(const uint8_t* data, size_t size);
	void Append(const std::vector<uint8_t>& bytes);
	bool TryPopFrame(std::vector<uint8_t>& outFrame);
	void Clear();
	size_t Size() const { return m_bytes.size(); }

private:
	std::vector<uint8_t> m_bytes;
};

} // namespace Network
} // namespace Runtime
