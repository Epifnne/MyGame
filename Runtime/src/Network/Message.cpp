#include "Network/Message.h"

namespace Runtime {
namespace Network {

MessageType Message::Type() const {
	return static_cast<MessageType>(header.type);
}

void Message::SetType(MessageType type) {
	header.type = static_cast<uint16_t>(type);
}

std::string Message::PayloadAsString() const {
	return std::string(payload.begin(), payload.end());
}

Message Message::FromText(MessageType type, std::string text, uint32_t sequence, uint16_t channel) {
	Message message;
	message.SetType(type);
	message.header.channel = channel;
	message.header.sequence = sequence;
	message.payload.assign(text.begin(), text.end());
	message.header.payloadSize = static_cast<uint32_t>(message.payload.size());
	return message;
}

Message Message::Ping(uint32_t sequence) {
	return FromText(MessageType::Ping, "ping", sequence, 0);
}

Message Message::Pong(uint32_t sequence) {
	return FromText(MessageType::Pong, "pong", sequence, 0);
}

} // namespace Network
} // namespace Runtime
