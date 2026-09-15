#pragma once

#include <algorithm>
#include <string>

#include <glm/vec2.hpp>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class ScrollView : public Widget {
public:
	explicit ScrollView(std::string id = {}) : Widget(std::move(id), "ScrollView") {}

	void SetContentSize(const glm::vec2& size) { m_contentSize = size; }
	glm::vec2 GetContentSize() const { return m_contentSize; }

	void SetScrollOffset(const glm::vec2& offset) {
		const Rect r = GetWorldRect();
		const glm::vec2 maxOffset{
			std::max(0.0f, m_contentSize.x - r.size.x),
			std::max(0.0f, m_contentSize.y - r.size.y)};
		m_scrollOffset.x = std::clamp(offset.x, 0.0f, maxOffset.x);
		m_scrollOffset.y = std::clamp(offset.y, 0.0f, maxOffset.y);
	}

	glm::vec2 GetScrollOffset() const { return m_scrollOffset; }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (e.type == UIEventType::ValueChanged && GetWorldRect().Contains(e.screenPosition)) {
			SetScrollOffset(glm::vec2(m_scrollOffset.x, m_scrollOffset.y - e.value * 12.0f));
			return true;
		}
		return false;
	}

private:
	glm::vec2 m_contentSize{0.0f, 0.0f};
	glm::vec2 m_scrollOffset{0.0f, 0.0f};
};

} // namespace UI
} // namespace Runtime
