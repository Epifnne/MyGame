#pragma once

#include <string>
#include <utility>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class InputField : public Widget {
public:
	explicit InputField(std::string id = {}) : Widget(std::move(id), "InputField") {}

	void SetText(std::string text) { m_text = std::move(text); }
	const std::string& GetText() const { return m_text; }

	void SetPlaceholder(std::string placeholder) { m_placeholder = std::move(placeholder); }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (e.type == UIEventType::PointerDown) {
			m_focused = GetWorldRect().Contains(e.screenPosition);
			return m_focused;
		}

		if (m_focused && e.type == UIEventType::TextInput) {
			m_text += e.text;
			return true;
		}

		if (m_focused && e.type == UIEventType::Blur) {
			m_focused = false;
			return true;
		}
		return false;
	}

	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand bg;
		bg.type = UIDrawType::Rect;
		bg.rect = GetWorldRect();
		bg.color = m_focused ? glm::vec4(0.15f, 0.2f, 0.28f, 1.0f) : glm::vec4(0.12f, 0.12f, 0.16f, 1.0f);
		bg.layer = GetLayer();
		renderer.Submit(bg);

		UIDrawCommand text;
		text.type = UIDrawType::Text;
		text.rect = GetWorldRect();
		text.text = m_text.empty() ? m_placeholder : m_text;
		text.color = m_text.empty() ? glm::vec4(0.6f, 0.6f, 0.6f, 1.0f) : glm::vec4(0.95f, 0.95f, 0.95f, 1.0f);
		text.layer = GetLayer() + 1;
		renderer.Submit(text);
	}

private:
	std::string m_text;
	std::string m_placeholder = "Input...";
	bool m_focused = false;
};

} // namespace UI
} // namespace Runtime
