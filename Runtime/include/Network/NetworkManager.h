#pragma once

#include <atomic>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "Network/Connection.h"

namespace Runtime {
namespace Network {

enum class NetworkEventType {
	Connected,
	Disconnected,
	MessageReceived,
	Error
};

struct NetworkEvent {
	NetworkEventType type = NetworkEventType::Error;
	Message message;
	std::string detail;
};

class NetworkManager {
public:
	NetworkManager();
	~NetworkManager();

	NetworkManager(const NetworkManager&) = delete;
	NetworkManager& operator=(const NetworkManager&) = delete;

	bool StartClient(const Endpoint& endpoint);
	void Stop();
	bool IsRunning() const { return m_running.load(); }
	bool IsConnected() const;

	bool Send(const Message& message);
	bool PollEvent(NetworkEvent& outEvent);

private:
	void WorkerMain();
	void PushEvent(NetworkEvent event);
	void TryReconnect();

	std::atomic<bool> m_running{false};
	std::atomic<bool> m_connected{false};
	Endpoint m_target;
	std::thread m_worker;

	mutable std::mutex m_connectionMutex;
	Connection m_connection;

	mutable std::mutex m_sendMutex;
	std::queue<Message> m_sendQueue;

	mutable std::mutex m_eventMutex;
	std::queue<NetworkEvent> m_eventQueue;

	uint32_t m_sequence = 1;
	int m_reconnectBudget = 1;
};

} // namespace Network
} // namespace Runtime
