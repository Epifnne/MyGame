#include <chrono>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "Network/Connection.h"
#include "Network/Message.h"
#include "Network/MessageSerializer.h"
#include "Network/NetworkManager.h"
#include "Network/PacketBuffer.h"
#include "Network/Socket.h"

using namespace Runtime::Network;

TEST(NetworkSerializerTest, RoundTripPreservesMessage) {
    Message src = Message::FromText(MessageType::Ping, "hello", 42, 7);
    std::vector<uint8_t> bytes = MessageSerializer::Serialize(src);

    auto decoded = MessageSerializer::Deserialize(bytes);
    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(decoded->header.type, src.header.type);
    EXPECT_EQ(decoded->header.channel, src.header.channel);
    EXPECT_EQ(decoded->header.sequence, src.header.sequence);
    EXPECT_EQ(decoded->PayloadAsString(), "hello");
}

TEST(PacketBufferTest, PopsFramesInOrder) {
    PacketBuffer buffer;

    auto appendFrame = [&buffer](const std::vector<uint8_t>& payload) {
        const uint32_t size = static_cast<uint32_t>(payload.size());
        buffer.Append(reinterpret_cast<const uint8_t*>(&size), sizeof(uint32_t));
        buffer.Append(payload);
    };

    appendFrame(std::vector<uint8_t>{1, 2, 3});
    appendFrame(std::vector<uint8_t>{9, 8});

    std::vector<uint8_t> frame;
    ASSERT_TRUE(buffer.TryPopFrame(frame));
    EXPECT_EQ(frame.size(), 3u);
    EXPECT_EQ(frame[0], 1);

    ASSERT_TRUE(buffer.TryPopFrame(frame));
    EXPECT_EQ(frame.size(), 2u);
    EXPECT_EQ(frame[0], 9);
}

TEST(NetworkSocketTest, TcpLoopbackPingPong) {
    ASSERT_TRUE(Socket::InitializeSockets());

    Socket listener;
    ASSERT_TRUE(listener.Create(Socket::Type::Tcp));
    ASSERT_TRUE(listener.Bind(Endpoint{"127.0.0.1", 46010}));
    ASSERT_TRUE(listener.Listen(4));
    ASSERT_TRUE(listener.SetNonBlocking(true));

    Connection client;
    ASSERT_TRUE(client.Connect(Endpoint{"127.0.0.1", 46010}));

    Connection serverConn;
    for (int i = 0; i < 100 && !serverConn.IsConnected(); ++i) {
        Endpoint remote;
        std::unique_ptr<Socket> accepted = listener.Accept(&remote);
        if (accepted) {
            serverConn.AttachAccepted(std::move(*accepted), remote);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    ASSERT_TRUE(serverConn.IsConnected());

    ASSERT_TRUE(client.SendMessage(Message::Ping(100)));

    bool replied = false;
    for (int i = 0; i < 200 && !replied; ++i) {
        std::vector<Message> serverMessages;
        serverConn.PollMessages(serverMessages);
        for (const Message& m : serverMessages) {
            if (m.Type() == MessageType::Ping) {
                Message pong = Message::Pong(m.header.sequence);
                ASSERT_TRUE(serverConn.SendMessage(pong));
            }
        }

        std::vector<Message> clientMessages;
        client.PollMessages(clientMessages);
        for (const Message& m : clientMessages) {
            if (m.Type() == MessageType::Pong && m.header.sequence == 100) {
                replied = true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    EXPECT_TRUE(replied);

    client.Disconnect();
    serverConn.Disconnect();
    Socket::ShutdownSockets();
}

TEST(NetworkManagerTest, ReconnectsOnceAfterServerDrop) {
    const Endpoint endpoint{"127.0.0.1", 46011};
    ASSERT_TRUE(Socket::InitializeSockets());

    std::thread serverThread([&]() {
        Socket listener;
        if (!listener.Create(Socket::Type::Tcp)) {
            return;
        }
        if (!listener.Bind(endpoint) || !listener.Listen(4) || !listener.SetNonBlocking(true)) {
            return;
        }

        auto serveOneClient = [&listener](int activeMs) {
            Connection serverConn;
            auto started = std::chrono::steady_clock::now();
            while (std::chrono::steady_clock::now() - started < std::chrono::seconds(3) && !serverConn.IsConnected()) {
                Endpoint remote;
                std::unique_ptr<Socket> accepted = listener.Accept(&remote);
                if (accepted) {
                    serverConn.AttachAccepted(std::move(*accepted), remote);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            if (!serverConn.IsConnected()) {
                return;
            }

            const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(activeMs);
            while (std::chrono::steady_clock::now() < until) {
                std::vector<Message> incoming;
                serverConn.PollMessages(incoming);
                for (const Message& message : incoming) {
                    if (message.Type() == MessageType::Ping) {
                        serverConn.SendMessage(Message::Pong(message.header.sequence));
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }

            // Force a drop so client reconnection logic is exercised.
            serverConn.Disconnect();
        };

        serveOneClient(400);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        serveOneClient(1000);
    });

    NetworkManager manager;
    ASSERT_TRUE(manager.StartClient(endpoint));

    int connectCount = 0;
    int pongCount = 0;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline && (connectCount < 2 || pongCount < 2)) {
        NetworkEvent event;
        while (manager.PollEvent(event)) {
            if (event.type == NetworkEventType::Connected) {
                ++connectCount;
                manager.Send(Message::Ping(100 + static_cast<uint32_t>(connectCount)));
            } else if (event.type == NetworkEventType::MessageReceived && event.message.Type() == MessageType::Pong) {
                ++pongCount;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    manager.Stop();
    if (serverThread.joinable()) {
        serverThread.join();
    }

    EXPECT_GE(connectCount, 2);
    EXPECT_GE(pongCount, 2);

    Socket::ShutdownSockets();
}
