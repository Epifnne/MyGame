#include "Network/Socket.h"

#include <atomic>
#include <cstring>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace Runtime {
namespace Network {

namespace {

std::atomic<int> g_socketInitRef{0};

#ifdef _WIN32
constexpr intptr_t kInvalidSocket = static_cast<intptr_t>(INVALID_SOCKET);
using SockLen = int;
using NativeSocket = SOCKET;
#else
constexpr intptr_t kInvalidSocket = static_cast<intptr_t>(-1);
using SockLen = socklen_t;
using NativeSocket = int;
#endif

inline NativeSocket ToNative(intptr_t handle) {
	return static_cast<NativeSocket>(handle);
}

void CloseNativeSocket(intptr_t handle) {
	if (handle == kInvalidSocket) {
		return;
	}
#ifdef _WIN32
	closesocket(ToNative(handle));
#else
	close(ToNative(handle));
#endif
}

bool ToSockAddr(const Endpoint& endpoint, sockaddr_in& addr) {
	std::memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(endpoint.port);
#ifdef _WIN32
	return inet_pton(AF_INET, endpoint.address.c_str(), &addr.sin_addr) == 1;
#else
	return inet_pton(AF_INET, endpoint.address.c_str(), &addr.sin_addr) == 1;
#endif
}

Endpoint FromSockAddr(const sockaddr_in& addr) {
	Endpoint endpoint;
	char ip[INET_ADDRSTRLEN] = {};
	inet_ntop(AF_INET, &addr.sin_addr, ip, static_cast<socklen_t>(sizeof(ip)));
	endpoint.address = ip;
	endpoint.port = ntohs(addr.sin_port);
	return endpoint;
}

} // namespace

Socket::Socket() = default;

Socket::~Socket() {
	Close();
}

Socket::Socket(intptr_t nativeHandle, Type type)
	: m_nativeHandle(nativeHandle), m_type(type) {
}

Socket::Socket(Socket&& other) noexcept {
	m_nativeHandle = other.m_nativeHandle;
	m_type = other.m_type;
	other.m_nativeHandle = kInvalidSocket;
}

Socket& Socket::operator=(Socket&& other) noexcept {
	if (this == &other) {
		return *this;
	}
	Close();
	m_nativeHandle = other.m_nativeHandle;
	m_type = other.m_type;
	other.m_nativeHandle = kInvalidSocket;
	return *this;
}

bool Socket::InitializeSockets() {
#ifdef _WIN32
	// Reference counting allows multiple subsystems/tests to call init safely.
	const int previous = g_socketInitRef.fetch_add(1);
	if (previous > 0) {
		return true;
	}
	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		g_socketInitRef.fetch_sub(1);
		return false;
	}
#else
	g_socketInitRef.fetch_add(1);
#endif
	return true;
}

void Socket::ShutdownSockets() {
	const int left = g_socketInitRef.fetch_sub(1) - 1;
	if (left > 0) {
		return;
	}
	g_socketInitRef.store(0);
#ifdef _WIN32
	WSACleanup();
#endif
}

bool Socket::Create(Type type) {
	Close();
	m_type = type;
	const int sockType = type == Type::Tcp ? SOCK_STREAM : SOCK_DGRAM;
	const int protocol = type == Type::Tcp ? IPPROTO_TCP : IPPROTO_UDP;
	m_nativeHandle = static_cast<intptr_t>(::socket(AF_INET, sockType, protocol));
	return IsValid();
}

void Socket::Close() {
	if (!IsValid()) {
		return;
	}
	CloseNativeSocket(m_nativeHandle);
	m_nativeHandle = kInvalidSocket;
}

bool Socket::IsValid() const {
	return m_nativeHandle != kInvalidSocket;
}

bool Socket::SetNonBlocking(bool enabled) {
	if (!IsValid()) {
		return false;
	}
#ifdef _WIN32
	// FIONBIO toggles blocking mode on Winsock sockets.
	u_long mode = enabled ? 1UL : 0UL;
	return ioctlsocket(ToNative(m_nativeHandle), FIONBIO, &mode) == 0;
#else
	// POSIX uses O_NONBLOCK on file descriptor flags.
	int flags = fcntl(ToNative(m_nativeHandle), F_GETFL, 0);
	if (flags < 0) {
		return false;
	}
	if (enabled) {
		flags |= O_NONBLOCK;
	} else {
		flags &= ~O_NONBLOCK;
	}
	return fcntl(ToNative(m_nativeHandle), F_SETFL, flags) == 0;
#endif
}

bool Socket::Bind(const Endpoint& endpoint) {
	if (!IsValid()) {
		return false;
	}

	sockaddr_in addr;
	if (!ToSockAddr(endpoint, addr)) {
		return false;
	}

	const int opt = 1;
	setsockopt(ToNative(m_nativeHandle), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&opt), sizeof(opt));
	return ::bind(ToNative(m_nativeHandle), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0;
}

bool Socket::Listen(int backlog) {
	if (!IsValid() || m_type != Type::Tcp) {
		return false;
	}
	return ::listen(ToNative(m_nativeHandle), backlog) == 0;
}

std::unique_ptr<Socket> Socket::Accept(Endpoint* outRemote) {
	if (!IsValid() || m_type != Type::Tcp) {
		return nullptr;
	}

	sockaddr_in remoteAddr;
	SockLen len = static_cast<SockLen>(sizeof(remoteAddr));
	const intptr_t accepted = static_cast<intptr_t>(::accept(ToNative(m_nativeHandle), reinterpret_cast<sockaddr*>(&remoteAddr), &len));
	if (accepted == kInvalidSocket) {
		return nullptr;
	}

	if (outRemote) {
		*outRemote = FromSockAddr(remoteAddr);
	}
	return std::unique_ptr<Socket>(new Socket(accepted, Type::Tcp));
}

bool Socket::Connect(const Endpoint& endpoint) {
	if (!IsValid()) {
		return false;
	}

	sockaddr_in addr;
	if (!ToSockAddr(endpoint, addr)) {
		return false;
	}

	return ::connect(ToNative(m_nativeHandle), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) == 0;
}

int Socket::Send(const uint8_t* data, size_t size) {
	if (!IsValid() || !data || size == 0) {
		return -1;
	}
	return static_cast<int>(::send(ToNative(m_nativeHandle), reinterpret_cast<const char*>(data), static_cast<int>(size), 0));
}

int Socket::Receive(uint8_t* data, size_t size) {
	if (!IsValid() || !data || size == 0) {
		return -1;
	}
	return static_cast<int>(::recv(ToNative(m_nativeHandle), reinterpret_cast<char*>(data), static_cast<int>(size), 0));
}

int Socket::SendTo(const Endpoint& endpoint, const uint8_t* data, size_t size) {
	if (!IsValid() || !data || size == 0 || m_type != Type::Udp) {
		return -1;
	}

	sockaddr_in addr;
	if (!ToSockAddr(endpoint, addr)) {
		return -1;
	}

	return static_cast<int>(::sendto(ToNative(m_nativeHandle), reinterpret_cast<const char*>(data), static_cast<int>(size), 0,
									  reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)));
}

int Socket::ReceiveFrom(Endpoint& from, uint8_t* data, size_t size) {
	if (!IsValid() || !data || size == 0 || m_type != Type::Udp) {
		return -1;
	}

	sockaddr_in addr;
	SockLen len = static_cast<SockLen>(sizeof(addr));
	const int received = static_cast<int>(::recvfrom(ToNative(m_nativeHandle), reinterpret_cast<char*>(data), static_cast<int>(size), 0,
													 reinterpret_cast<sockaddr*>(&addr), &len));
	if (received > 0) {
		from = FromSockAddr(addr);
	}
	return received;
}

} // namespace Network
} // namespace Runtime
