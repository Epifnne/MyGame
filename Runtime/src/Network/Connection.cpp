#include "Network/Connection.h"

#include "Network/MessageSerializer.h"

namespace Runtime {
namespace Network {

namespace {

std::vector<uint8_t> BuildFrame(const Message& message) {
	const std::vector<uint8_t> wire = MessageSerializer::Serialize(message);
	const uint32_t size = static_cast<uint32_t>(wire.size());

	// Transport frame: [4-byte little-endian payload size][wire payload].
	std::vector<uint8_t> frame;
	frame.reserve(sizeof(uint32_t) + wire.size());
	frame.push_back(static_cast<uint8_t>(size & 0xFFu));
	frame.push_back(static_cast<uint8_t>((size >> 8u) & 0xFFu));
	frame.push_back(static_cast<uint8_t>((size >> 16u) & 0xFFu));
	frame.push_back(static_cast<uint8_t>((size >> 24u) & 0xFFu));
	frame.insert(frame.end(), wire.begin(), wire.end());
	return frame;
}

} // namespace

bool Connection::Connect(const Endpoint& endpoint) {
	m_state = State::Connecting;
	m_remote = endpoint;

	if (!m_socket.Create(Socket::Type::Tcp)) {
		m_state = State::Disconnected;
		return false;
	}

	if (!m_socket.Connect(endpoint)) {
		m_socket.Close();
		m_state = State::Disconnected;
		return false;
	}

	m_socket.SetNonBlocking(true);
	m_state = State::Connected;
	return true;
}

void Connection::AttachAccepted(Socket&& socket, const Endpoint& remote) {
	m_socket = std::move(socket);
	m_remote = remote;
	m_socket.SetNonBlocking(true);
	m_state = State::Connected;
}

void Connection::Disconnect() {
	m_socket.Close();
	m_receiveBuffer.Clear();
	m_state = State::Disconnected;
}

bool Connection::SendMessage(const Message& message) {
	if (!IsConnected()) {
		return false;
	}
	std::vector<uint8_t> frame = BuildFrame(message);
	const int sent = m_socket.Send(frame.data(), frame.size());
	return sent == static_cast<int>(frame.size());
}

bool Connection::PollMessages(std::vector<Message>& outMessages) {
	if (!IsConnected()) {
		return false;
	}

	uint8_t buffer[4096];
	bool gotAny = false;

	while (true) {
		const int received = m_socket.Receive(buffer, sizeof(buffer));
		if (received == 0) {
			// Peer performed an orderly shutdown.
			Disconnect();
			break;
		}
		if (received < 0) {
			// Non-blocking socket has no more bytes available right now.
			break;
		}

		gotAny = true;
		m_receiveBuffer.Append(buffer, static_cast<size_t>(received));

		std::vector<uint8_t> frame;
		while (m_receiveBuffer.TryPopFrame(frame)) {
			auto message = MessageSerializer::Deserialize(frame);
			if (message.has_value()) {
				// Keep processing contiguous buffered frames in one poll tick.
				outMessages.push_back(std::move(message.value()));
			}
		}
	}

	return gotAny;
}

} // namespace Network
} // namespace Runtime
