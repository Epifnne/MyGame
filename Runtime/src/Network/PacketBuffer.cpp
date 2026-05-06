#include "Network/PacketBuffer.h"

#include <cstring>

namespace Runtime {
namespace Network {

void PacketBuffer::Append(const uint8_t* data, size_t size) {
	if (!data || size == 0) {
		return;
	}
	m_bytes.insert(m_bytes.end(), data, data + size);
}

void PacketBuffer::Append(const std::vector<uint8_t>& bytes) {
	if (bytes.empty()) {
		return;
	}
	m_bytes.insert(m_bytes.end(), bytes.begin(), bytes.end());
}

bool PacketBuffer::TryPopFrame(std::vector<uint8_t>& outFrame) {
	// TCP is a byte stream, so each message is prefixed with a 4-byte frame length.
	if (m_bytes.size() < sizeof(uint32_t)) {
		return false;
	}

	const uint32_t frameSize = static_cast<uint32_t>(m_bytes[0]) |
							   (static_cast<uint32_t>(m_bytes[1]) << 8u) |
							   (static_cast<uint32_t>(m_bytes[2]) << 16u) |
							   (static_cast<uint32_t>(m_bytes[3]) << 24u);

	const size_t totalSize = sizeof(uint32_t) + static_cast<size_t>(frameSize);
	if (m_bytes.size() < totalSize) {
		// Not enough bytes for a complete frame yet.
		return false;
	}

	// Extract one complete frame and keep remainder for subsequent reads.
	outFrame.assign(m_bytes.begin() + sizeof(uint32_t), m_bytes.begin() + totalSize);
	m_bytes.erase(m_bytes.begin(), m_bytes.begin() + totalSize);
	return true;
}

void PacketBuffer::Clear() {
	m_bytes.clear();
}

} // namespace Network
} // namespace Runtime
