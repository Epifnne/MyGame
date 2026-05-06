#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace Runtime {
namespace Network {

struct Endpoint {
	std::string address;
	uint16_t port = 0;
};

class Socket {
public:
	enum class Type {
		Tcp,
		Udp
	};

	Socket();
	~Socket();

	Socket(const Socket&) = delete;
	Socket& operator=(const Socket&) = delete;

	Socket(Socket&& other) noexcept;
	Socket& operator=(Socket&& other) noexcept;

	static bool InitializeSockets();
	static void ShutdownSockets();

	bool Create(Type type);
	void Close();
	bool IsValid() const;
	bool SetNonBlocking(bool enabled);

	bool Bind(const Endpoint& endpoint);
	bool Listen(int backlog = 16);
	std::unique_ptr<Socket> Accept(Endpoint* outRemote = nullptr);
	bool Connect(const Endpoint& endpoint);

	int Send(const uint8_t* data, size_t size);
	int Receive(uint8_t* data, size_t size);

	int SendTo(const Endpoint& endpoint, const uint8_t* data, size_t size);
	int ReceiveFrom(Endpoint& from, uint8_t* data, size_t size);

private:
	friend class Connection;
	explicit Socket(intptr_t nativeHandle, Type type);

	intptr_t m_nativeHandle = -1;
	Type m_type = Type::Tcp;
};

} // namespace Network
} // namespace Runtime
