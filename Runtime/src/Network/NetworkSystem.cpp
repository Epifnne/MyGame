#include "Network/NetworkSystem.h"

namespace Runtime {
namespace Network {

void NetworkSystem::Update(ECS::Registry& registry, float dt) {
	(void)registry;
	(void)dt;

	if (!m_manager) {
		return;
	}

	NetworkEvent event;
	while (m_manager->PollEvent(event)) {
		switch (event.type) {
		case NetworkEventType::Connected:
			if (m_onConnected) {
				m_onConnected();
			}
			break;
		case NetworkEventType::Disconnected:
			if (m_onDisconnected) {
				m_onDisconnected();
			}
			break;
		case NetworkEventType::MessageReceived:
			if (m_onMessage) {
				m_onMessage(event.message);
			}
			break;
		case NetworkEventType::Error:
			if (m_onError) {
				m_onError(event.detail.c_str());
			}
			break;
		}
	}
}

} // namespace Network
} // namespace Runtime
