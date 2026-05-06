#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace Runtime {
namespace Network {

enum class TransportProtocol : uint8_t {
	Tcp = 0,
	Udp = 1
};

enum class MessageType : uint16_t {
	Invalid = 0,
	Connect = 1,
	Ping = 2,
	Pong = 3,
	Disconnect = 4,
	Heartbeat = 5,
	UserBegin = 1000
};

struct MessageHeader {
	uint16_t type = static_cast<uint16_t>(MessageType::Invalid);
	uint16_t channel = 0;
	uint32_t sequence = 0;
	uint32_t payloadSize = 0;
};

struct Message {
	MessageHeader header;
	std::vector<uint8_t> payload;

	MessageType Type() const;
	void SetType(MessageType type);
	std::string PayloadAsString() const;

	static Message FromText(MessageType type, std::string text, uint32_t sequence = 0, uint16_t channel = 0);
	static Message Ping(uint32_t sequence = 0);
	static Message Pong(uint32_t sequence = 0);
};

} // namespace Network
} // namespace Runtime
