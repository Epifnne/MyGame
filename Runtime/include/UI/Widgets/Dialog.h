#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

struct DialogButton {
	std::string label;
	std::function<void()> onClick;
};

class Dialog : public Widget {
public:
	explicit Dialog(std::string id = {}) : Widget(std::move(id), "Dialog") {}

	void SetTitle(std::string title) { m_title = std::move(title); }
	void SetMessage(std::string message) { m_message = std::move(message); }
	void SetButtons(std::vector<DialogButton> buttons) { m_buttons = std::move(buttons); }

	void Open() { m_open = true; }
	void Close() { m_open = false; }
	bool IsOpen() const { return m_open; }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (!m_open || e.type != UIEventType::PointerDown || !GetWorldRect().Contains(e.screenPosition)) {
			return false;
		}

		if (!m_buttons.empty()) {
			// Minimal behavior: first button acts as primary action.
			if (m_buttons.front().onClick) {
				m_buttons.front().onClick();
			}
		}
		return true;
	}

	void OnRender(UIRenderer& renderer) const override {
		if (!m_open) {
			return;
		}

		UIDrawCommand bg;
		bg.type = UIDrawType::Rect;
		bg.rect = GetWorldRect();
		bg.color = glm::vec4(0.08f, 0.08f, 0.1f, 0.95f);
		bg.layer = GetLayer();
		renderer.Submit(bg);

		Rect titleRect = GetWorldRect();
		titleRect.size.y = 28.0f;
		UIDrawCommand title;
		title.type = UIDrawType::Text;
		title.rect = titleRect;
		title.text = m_title;
		title.color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
		title.layer = GetLayer() + 1;
		renderer.Submit(title);

		Rect msgRect = GetWorldRect();
		msgRect.position.y += 30.0f;
		msgRect.size.y -= 60.0f;
		UIDrawCommand msg;
		msg.type = UIDrawType::Text;
		msg.rect = msgRect;
		msg.text = m_message;
		msg.color = glm::vec4(0.88f, 0.88f, 0.9f, 1.0f);
		msg.layer = GetLayer() + 1;
		renderer.Submit(msg);
	}

private:
	std::string m_title = "Dialog";
	std::string m_message;
	std::vector<DialogButton> m_buttons;
	bool m_open = false;
};

} // namespace UI
} // namespace Runtime
