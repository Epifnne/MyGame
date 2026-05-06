#include "Network/MessageSerializer.h"

#include <cstring>

namespace Runtime {
namespace Network {

namespace {

// Wire format uses little-endian fixed-width integers to keep parsing deterministic.
void WriteU16(std::vector<uint8_t>& out, uint16_t value) {
	out.push_back(static_cast<uint8_t>(value & 0xFFu));
	out.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
}

void WriteU32(std::vector<uint8_t>& out, uint32_t value) {
	out.push_back(static_cast<uint8_t>(value & 0xFFu));
	out.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
	out.push_back(static_cast<uint8_t>((value >> 16u) & 0xFFu));
	out.push_back(static_cast<uint8_t>((value >> 24u) & 0xFFu));
}

uint16_t ReadU16(const uint8_t* data) {
	return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8u);
}

uint32_t ReadU32(const uint8_t* data) {
	return static_cast<uint32_t>(data[0]) |
		   (static_cast<uint32_t>(data[1]) << 8u) |
		   (static_cast<uint32_t>(data[2]) << 16u) |
		   (static_cast<uint32_t>(data[3]) << 24u);
}

} // namespace

std::vector<uint8_t> MessageSerializer::Serialize(const Message& message) {
	std::vector<uint8_t> bytes;
	bytes.reserve(WireHeaderSize + message.payload.size());

	// Header is always 12 bytes: type(2), channel(2), sequence(4), payloadSize(4).
	WriteU16(bytes, message.header.type);
	WriteU16(bytes, message.header.channel);
	WriteU32(bytes, message.header.sequence);
	WriteU32(bytes, static_cast<uint32_t>(message.payload.size()));
	bytes.insert(bytes.end(), message.payload.begin(), message.payload.end());
	return bytes;
}

std::optional<Message> MessageSerializer::Deserialize(const uint8_t* data, size_t size) {
	if (!data || size < WireHeaderSize) {
		return std::nullopt;
	}

	Message message;
	message.header.type = ReadU16(data);
	message.header.channel = ReadU16(data + 2);
	message.header.sequence = ReadU32(data + 4);
	message.header.payloadSize = ReadU32(data + 8);

	const size_t expected = WireHeaderSize + static_cast<size_t>(message.header.payloadSize);
	// Stream input may provide partial frames; caller retries after accumulating more bytes.
	if (size < expected) {
		return std::nullopt;
	}

	message.payload.resize(message.header.payloadSize);
	if (message.header.payloadSize > 0) {
		std::memcpy(message.payload.data(), data + WireHeaderSize, message.header.payloadSize);
	}
	return message;
}

std::optional<Message> MessageSerializer::Deserialize(const std::vector<uint8_t>& bytes) {
	return Deserialize(bytes.data(), bytes.size());
}

} // namespace Network
} // namespace Runtime
