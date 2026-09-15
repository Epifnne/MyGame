#pragma once

#include <string>
#include <utility>
#include <vector>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class TabView : public Widget {
public:
	explicit TabView(std::string id = {}) : Widget(std::move(id), "TabView") {}

	void SetTabs(std::vector<std::string> tabs) { m_tabs = std::move(tabs); }
	const std::vector<std::string>& GetTabs() const { return m_tabs; }

	void SetActiveTab(int index) {
		if (index >= 0 && index < static_cast<int>(m_tabs.size())) {
			m_activeIndex = index;
		}
	}

	int GetActiveTab() const { return m_activeIndex; }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (e.type != UIEventType::PointerDown || !GetWorldRect().Contains(e.screenPosition) || m_tabs.empty()) {
			return false;
		}
		const float tabWidth = GetWorldRect().size.x / static_cast<float>(m_tabs.size());
		const float localX = e.screenPosition.x - GetWorldRect().position.x;
		SetActiveTab(static_cast<int>(localX / tabWidth));
		return true;
	}

	void OnRender(UIRenderer& renderer) const override {
		if (m_tabs.empty()) {
			return;
		}

		const float tabWidth = GetWorldRect().size.x / static_cast<float>(m_tabs.size());
		for (size_t i = 0; i < m_tabs.size(); ++i) {
			Rect tabRect = GetWorldRect();
			tabRect.position.x += static_cast<float>(i) * tabWidth;
			tabRect.size.x = tabWidth;
			tabRect.size.y = 28.0f;

			UIDrawCommand bg;
			bg.type = UIDrawType::Rect;
			bg.rect = tabRect;
			bg.color = (static_cast<int>(i) == m_activeIndex) ? glm::vec4(0.22f, 0.46f, 0.78f, 1.0f)
																: glm::vec4(0.16f, 0.16f, 0.2f, 1.0f);
			bg.layer = GetLayer();
			renderer.Submit(bg);

			UIDrawCommand txt;
			txt.type = UIDrawType::Text;
			txt.rect = tabRect;
			txt.text = m_tabs[i];
			txt.color = glm::vec4(0.95f, 0.95f, 0.95f, 1.0f);
			txt.layer = GetLayer() + 1;
			renderer.Submit(txt);
		}
	}

private:
	std::vector<std::string> m_tabs;
	int m_activeIndex = 0;
};

} // namespace UI
} // namespace Runtime
