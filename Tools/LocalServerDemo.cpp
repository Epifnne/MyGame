#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

#include "Network/Connection.h"
#include "Network/Message.h"
#include "Network/Socket.h"

int main() {
    using namespace Runtime::Network;

    if (!Socket::InitializeSockets()) {
        std::cerr << "[LocalServer] failed to initialize sockets" << std::endl;
        return 1;
    }

    Socket listener;
    if (!listener.Create(Socket::Type::Tcp)) {
        std::cerr << "[LocalServer] failed to create TCP socket" << std::endl;
        Socket::ShutdownSockets();
        return 2;
    }

    Endpoint endpoint{"127.0.0.1", 45000};
    if (!listener.Bind(endpoint)) {
        std::cerr << "[LocalServer] bind failed for " << endpoint.address << ':' << endpoint.port << std::endl;
        Socket::ShutdownSockets();
        return 3;
    }

    if (!listener.Listen(8)) {
        std::cerr << "[LocalServer] listen failed" << std::endl;
        Socket::ShutdownSockets();
        return 4;
    }

    listener.SetNonBlocking(true);
    std::cout << "[LocalServer] listening on " << endpoint.address << ':' << endpoint.port << std::endl;

    Connection client;
    auto lastMessageAt = std::chrono::steady_clock::now();

    while (true) {
        if (!client.IsConnected()) {
            Endpoint remote;
            std::unique_ptr<Socket> accepted = listener.Accept(&remote);
            if (accepted) {
                client.AttachAccepted(std::move(*accepted), remote);
                std::cout << "[LocalServer] client connected from " << remote.address << ':' << remote.port << std::endl;
                lastMessageAt = std::chrono::steady_clock::now();
            }
        }

        if (client.IsConnected()) {
            std::vector<Message> incoming;
            client.PollMessages(incoming);

            for (const Message& msg : incoming) {
                lastMessageAt = std::chrono::steady_clock::now();

                if (msg.Type() == MessageType::Connect) {
                    std::cout << "[LocalServer] handshake: " << msg.PayloadAsString() << std::endl;
                    continue;
                }

                if (msg.Type() == MessageType::Ping) {
                    Message pong = Message::Pong(msg.header.sequence);
                    client.SendMessage(pong);
                    std::cout << "[LocalServer] ping -> pong seq=" << msg.header.sequence << std::endl;
                    continue;
                }

                if (msg.Type() == MessageType::Disconnect) {
                    std::cout << "[LocalServer] disconnect requested" << std::endl;
                    client.Disconnect();
                    Socket::ShutdownSockets();
                    return 0;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            const auto idleSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - lastMessageAt).count();
            if (idleSeconds > 15) {
                std::cout << "[LocalServer] idle timeout, closing session" << std::endl;
                client.Disconnect();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
