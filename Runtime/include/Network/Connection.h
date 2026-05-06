#pragma once

#include <cstdint>
#include <vector>

#include "Network/Message.h"
#include "Network/PacketBuffer.h"
#include "Network/Socket.h"

namespace Runtime {
namespace Network {

class Connection {
public:
	enum class State {
		Disconnected,
		Connecting,
		Connected
	};

	Connection() = default;
	~Connection() = default;

	bool Connect(const Endpoint& endpoint);
	void AttachAccepted(Socket&& socket, const Endpoint& remote);
	void Disconnect();

	bool IsConnected() const { return m_state == State::Connected; }
	State GetState() const { return m_state; }
	Endpoint RemoteEndpoint() const { return m_remote; }

	bool SendMessage(const Message& message);
	bool PollMessages(std::vector<Message>& outMessages);

private:
	Socket m_socket;
	PacketBuffer m_receiveBuffer;
	Endpoint m_remote;
	State m_state = State::Disconnected;
};

} // namespace Network
} // namespace Runtime
