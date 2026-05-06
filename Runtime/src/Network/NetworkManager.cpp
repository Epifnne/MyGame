#include "Network/NetworkManager.h"

#include <chrono>

namespace Runtime {
namespace Network {

NetworkManager::NetworkManager() {
	Socket::InitializeSockets();
}

NetworkManager::~NetworkManager() {
	Stop();
	Socket::ShutdownSockets();
}

bool NetworkManager::StartClient(const Endpoint& endpoint) {
	if (m_running.load()) {
		return false;
	}

	m_target = endpoint;
	m_reconnectBudget = 1;
	m_running.store(true);
	m_worker = std::thread(&NetworkManager::WorkerMain, this);
	return true;
}

void NetworkManager::Stop() {
	if (!m_running.exchange(false)) {
		return;
	}

	if (m_worker.joinable()) {
		m_worker.join();
	}

	std::lock_guard<std::mutex> lock(m_connectionMutex);
	m_connection.Disconnect();
	m_connected.store(false);
}

bool NetworkManager::IsConnected() const {
	return m_connected.load();
}

bool NetworkManager::Send(const Message& message) {
	if (!m_running.load()) {
		return false;
	}

	Message copy = message;
	if (copy.header.sequence == 0) {
		copy.header.sequence = m_sequence++;
	}

	std::lock_guard<std::mutex> lock(m_sendMutex);
	m_sendQueue.push(std::move(copy));
	return true;
}

bool NetworkManager::PollEvent(NetworkEvent& outEvent) {
	std::lock_guard<std::mutex> lock(m_eventMutex);
	if (m_eventQueue.empty()) {
		return false;
	}
	outEvent = std::move(m_eventQueue.front());
	m_eventQueue.pop();
	return true;
}

void NetworkManager::WorkerMain() {
	auto lastHeartbeat = std::chrono::steady_clock::now();
	{
		std::lock_guard<std::mutex> lock(m_connectionMutex);
		if (!m_connection.Connect(m_target)) {
			m_connected.store(false);
			PushEvent(NetworkEvent{NetworkEventType::Error, {}, "initial connect failed"});
			TryReconnect();
		} else {
			m_connected.store(true);
			PushEvent(NetworkEvent{NetworkEventType::Connected, {}, "connected"});
			// Initial handshake payload so server can identify a fresh client session.
			Message hello = Message::FromText(MessageType::Connect, "hello");
			m_connection.SendMessage(hello);
		}
	}

	while (m_running.load()) {
		{
			std::lock_guard<std::mutex> lock(m_connectionMutex);
			if (m_connection.IsConnected()) {
				const auto now = std::chrono::steady_clock::now();
				if (now - lastHeartbeat >= std::chrono::milliseconds(500)) {
					// Application-level heartbeat keeps liveness detection independent of TCP keepalive.
					Message heartbeat = Message::FromText(MessageType::Heartbeat, "hb");
					if (!m_connection.SendMessage(heartbeat)) {
						m_connection.Disconnect();
						m_connected.store(false);
						PushEvent(NetworkEvent{NetworkEventType::Disconnected, {}, "heartbeat send failed"});
						TryReconnect();
						continue;
					}
					lastHeartbeat = now;
				}

				std::vector<Message> incoming;
				m_connection.PollMessages(incoming);
				if (!m_connection.IsConnected()) {
					// Any receive-side disconnect transitions to event + reconnection path.
					m_connected.store(false);
					PushEvent(NetworkEvent{NetworkEventType::Disconnected, {}, "remote closed"});
					TryReconnect();
					continue;
				}
				for (auto& message : incoming) {
					PushEvent(NetworkEvent{NetworkEventType::MessageReceived, message, {}});
				}

				std::queue<Message> pending;
				{
					std::lock_guard<std::mutex> sendLock(m_sendMutex);
					// Swap queue to minimize lock hold time on producer threads.
					pending.swap(m_sendQueue);
				}

				while (!pending.empty()) {
					if (!m_connection.SendMessage(pending.front())) {
						m_connection.Disconnect();
						m_connected.store(false);
						PushEvent(NetworkEvent{NetworkEventType::Disconnected, {}, "send failed"});
						TryReconnect();
						break;
					}
					pending.pop();
				}
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(2));
	}
}

void NetworkManager::PushEvent(NetworkEvent event) {
	std::lock_guard<std::mutex> lock(m_eventMutex);
	m_eventQueue.push(std::move(event));
}

void NetworkManager::TryReconnect() {
	if (m_reconnectBudget <= 0) {
		return;
	}
	--m_reconnectBudget;

	// Bounded retry window prevents endless spin when endpoint is unavailable.
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
	while (m_running.load() && !m_connection.IsConnected() && std::chrono::steady_clock::now() < deadline) {
		if (m_connection.Connect(m_target)) {
			m_connected.store(true);
			PushEvent(NetworkEvent{NetworkEventType::Connected, {}, "reconnected"});
			return;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	PushEvent(NetworkEvent{NetworkEventType::Error, {}, "reconnect exhausted"});
}

} // namespace Network
} // namespace Runtime
