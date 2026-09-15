#pragma once

#include <string>

#include <glm/vec2.hpp>

namespace Runtime {
namespace UI {

class Widget;

enum class UIEventType {
	PointerMove,
	PointerDown,
	PointerUp,
	Click,
	HoverEnter,
	HoverLeave,
	Focus,
	Blur,
	TextInput,
	ValueChanged
};

struct UIEvent {
	UIEventType type = UIEventType::PointerMove;
	glm::vec2 screenPosition{0.0f, 0.0f};
	int pointerButton = 0;
	float value = 0.0f;
	std::string text;
	Widget* target = nullptr;
};

struct UIEventMessage {
	UIEvent event;
	std::string widgetId;
};

} // namespace UI
} // namespace Runtime
