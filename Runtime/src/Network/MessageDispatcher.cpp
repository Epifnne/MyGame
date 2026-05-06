#include "Network/MessageDispatcher.h"

namespace Runtime {
namespace Network {

void MessageDispatcher::RegisterHandler(MessageType type, Handler handler) {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_handlers[static_cast<uint16_t>(type)] = std::move(handler);
}

bool MessageDispatcher::Dispatch(const Message& message) const {
	Handler handler;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto it = m_handlers.find(message.header.type);
		if (it == m_handlers.end()) {
			return false;
		}
		handler = it->second;
	}
	handler(message);
	return true;
}

void MessageDispatcher::Clear() {
	std::lock_guard<std::mutex> lock(m_mutex);
	m_handlers.clear();
}

} // namespace Network
} // namespace Runtime
