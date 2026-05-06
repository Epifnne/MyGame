#pragma once

#include <cstdint>
#include <vector>

namespace Runtime {
namespace Network {

struct StateSnapshot {
	uint32_t tick = 0;
	std::vector<uint8_t> payload;
};

} // namespace Network
} // namespace Runtime
