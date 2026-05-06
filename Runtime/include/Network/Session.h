#pragma once

#include <chrono>

namespace Runtime {
namespace Network {

class Session {
public:
	void MarkConnected();
	void MarkHeartbeat();
	void SetTimeoutSeconds(float seconds) { m_timeoutSeconds = seconds; }
	bool IsTimedOut() const;

private:
	std::chrono::steady_clock::time_point m_lastSeen{};
	float m_timeoutSeconds = 3.0f;
};

} // namespace Network
} // namespace Runtime
