#include "Network/Session.h"

namespace Runtime {
namespace Network {

void Session::MarkConnected() {
	m_lastSeen = std::chrono::steady_clock::now();
}

void Session::MarkHeartbeat() {
	m_lastSeen = std::chrono::steady_clock::now();
}

bool Session::IsTimedOut() const {
	if (m_lastSeen.time_since_epoch().count() == 0) {
		return false;
	}

	const auto now = std::chrono::steady_clock::now();
	const auto elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(now - m_lastSeen).count();
	return elapsed > m_timeoutSeconds;
}

} // namespace Network
} // namespace Runtime
