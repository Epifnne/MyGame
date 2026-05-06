#pragma once

#include <string>

namespace Runtime {
namespace Network {

class Channel {
public:
	enum class Status {
		Disabled,
		NotImplemented
	};

	Status CurrentStatus() const { return Status::NotImplemented; }
	std::string Describe() const { return "UDP reliable channel is reserved for phase 2"; }
};

} // namespace Network
} // namespace Runtime
