#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "Network/Message.h"

namespace Runtime {
namespace Network {

class MessageSerializer {
public:
	static std::vector<uint8_t> Serialize(const Message& message);
	static std::optional<Message> Deserialize(const uint8_t* data, size_t size);
	static std::optional<Message> Deserialize(const std::vector<uint8_t>& bytes);

	static constexpr uint32_t WireHeaderSize = sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t);
};

} // namespace Network
} // namespace Runtime
