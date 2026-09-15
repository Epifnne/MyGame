#pragma once

#include <algorithm>
#include <string>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class ProgressBar : public Widget {
public:
	explicit ProgressBar(std::string id = {}) : Widget(std::move(id), "ProgressBar") {}

	void SetProgress(float progress) { m_progress = std::clamp(progress, 0.0f, 1.0f); }
	float GetProgress() const { return m_progress; }

protected:
	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand bg;
		bg.type = UIDrawType::Rect;
		bg.rect = GetWorldRect();
		bg.color = glm::vec4(0.16f, 0.16f, 0.18f, 1.0f);
		bg.layer = GetLayer();
		renderer.Submit(bg);

		Rect fill = GetWorldRect();
		fill.size.x *= m_progress;

		UIDrawCommand fg;
		fg.type = UIDrawType::Rect;
		fg.rect = fill;
		fg.color = glm::vec4(0.2f, 0.6f, 0.92f, 1.0f);
		fg.layer = GetLayer() + 1;
		renderer.Submit(fg);
	}

private:
	float m_progress = 0.0f;
};

} // namespace UI
} // namespace Runtime
