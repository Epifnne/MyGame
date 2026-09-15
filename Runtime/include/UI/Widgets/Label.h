#pragma once

#include <string>

#include <glm/vec4.hpp>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class Label : public Widget {
public:
	explicit Label(std::string id = {}) : Widget(std::move(id), "Label") {}

	void SetText(std::string text) { m_text = std::move(text); }
	const std::string& GetText() const { return m_text; }

	void SetColor(const glm::vec4& color) { m_color = color; }
	const glm::vec4& GetColor() const { return m_color; }

protected:
	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand cmd;
		cmd.type = UIDrawType::Text;
		cmd.rect = GetWorldRect();
		cmd.color = m_color;
		cmd.text = m_text;
		cmd.layer = GetLayer();
		renderer.Submit(cmd);
	}

private:
	std::string m_text;
	glm::vec4 m_color{1.0f, 1.0f, 1.0f, 1.0f};
};

} // namespace UI
} // namespace Runtime
