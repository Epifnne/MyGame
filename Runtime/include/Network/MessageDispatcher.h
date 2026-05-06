#pragma once

#include <functional>
#include <mutex>
#include <unordered_map>

#include "Network/Message.h"

namespace Runtime {
namespace Network {

class MessageDispatcher {
public:
	using Handler = std::function<void(const Message&)>;

	void RegisterHandler(MessageType type, Handler handler);
	bool Dispatch(const Message& message) const;
	void Clear();

private:
	mutable std::mutex m_mutex;
	std::unordered_map<uint16_t, Handler> m_handlers;
};

} // namespace Network
} // namespace Runtime
