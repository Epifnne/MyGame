#pragma once

#include <string>

#include <glm/vec4.hpp>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class Panel : public Widget {
public:
	explicit Panel(std::string id = {}) : Widget(std::move(id), "Panel") {}

	void SetBackground(const glm::vec4& color) { m_background = color; }
	void SetBorder(const glm::vec4& color) { m_border = color; }

protected:
	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand bg;
		bg.type = UIDrawType::Rect;
		bg.rect = GetWorldRect();
		bg.color = m_background;
		bg.layer = GetLayer();
		renderer.Submit(bg);

		UIDrawCommand border;
		border.type = UIDrawType::Rect;
		border.rect = GetWorldRect();
		border.color = m_border;
		border.layer = GetLayer() + 1;
		renderer.Submit(border);
	}

private:
	glm::vec4 m_background{0.1f, 0.1f, 0.12f, 0.88f};
	glm::vec4 m_border{0.5f, 0.5f, 0.55f, 0.22f};
};

} // namespace UI
} // namespace Runtime
