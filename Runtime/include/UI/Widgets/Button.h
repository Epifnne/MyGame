#pragma once

#include <functional>
#include <string>

#include <glm/vec4.hpp>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class Button : public Widget {
public:
	explicit Button(std::string id = {}) : Widget(std::move(id), "Button") {}

	void SetText(std::string text) { m_text = std::move(text); }
	const std::string& GetText() const { return m_text; }

	void SetOnClick(std::function<void()> onClick) { m_onClick = std::move(onClick); }
	void SetColors(const glm::vec4& normal, const glm::vec4& hover, const glm::vec4& pressed) {
		m_normalColor = normal;
		m_hoverColor = hover;
		m_pressedColor = pressed;
	}

protected:
	bool OnEvent(const UIEvent& e) override {
		const bool contains = GetWorldRect().Contains(e.screenPosition);
		if (e.type == UIEventType::PointerDown && contains) {
			m_pressed = true;
			return true;
		}
		if (e.type == UIEventType::PointerUp) {
			const bool wasPressed = m_pressed;
			m_pressed = false;
			if (wasPressed && contains) {
				if (m_onClick) {
					m_onClick();
				}
				return true;
			}
		}
		if (e.type == UIEventType::PointerMove) {
			m_hovered = contains;
		}
		return false;
	}

	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand bg;
		bg.type = UIDrawType::Rect;
		bg.rect = GetWorldRect();
		bg.layer = GetLayer();
		bg.color = m_pressed ? m_pressedColor : (m_hovered ? m_hoverColor : m_normalColor);
		renderer.Submit(bg);

		UIDrawCommand text;
		text.type = UIDrawType::Text;
		text.rect = GetWorldRect();
		text.text = m_text;
		text.color = glm::vec4(0.95f, 0.95f, 0.95f, 1.0f);
		text.layer = GetLayer() + 1;
		renderer.Submit(text);
	}

private:
	std::string m_text = "Button";
	std::function<void()> m_onClick;
	bool m_hovered = false;
	bool m_pressed = false;

	glm::vec4 m_normalColor{0.2f, 0.2f, 0.24f, 1.0f};
	glm::vec4 m_hoverColor{0.28f, 0.28f, 0.34f, 1.0f};
	glm::vec4 m_pressedColor{0.15f, 0.46f, 0.78f, 1.0f};
};

} // namespace UI
} // namespace Runtime
