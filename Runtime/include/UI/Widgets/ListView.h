#pragma once

#include <string>
#include <utility>
#include <vector>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class ListView : public Widget {
public:
	explicit ListView(std::string id = {}) : Widget(std::move(id), "ListView") {}

	void SetItems(std::vector<std::string> items) { m_items = std::move(items); }
	const std::vector<std::string>& GetItems() const { return m_items; }

	void SetSelectedIndex(int index) {
		if (index >= 0 && index < static_cast<int>(m_items.size())) {
			m_selectedIndex = index;
		}
	}

	int GetSelectedIndex() const { return m_selectedIndex; }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (e.type != UIEventType::PointerDown || !GetWorldRect().Contains(e.screenPosition)) {
			return false;
		}

		const float itemHeight = 20.0f;
		const float localY = e.screenPosition.y - GetWorldRect().position.y;
		const int index = static_cast<int>(localY / itemHeight);
		SetSelectedIndex(index);
		return true;
	}

	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand bg;
		bg.type = UIDrawType::Rect;
		bg.rect = GetWorldRect();
		bg.color = glm::vec4(0.11f, 0.11f, 0.13f, 0.95f);
		bg.layer = GetLayer();
		renderer.Submit(bg);

		const float itemHeight = 20.0f;
		for (size_t i = 0; i < m_items.size(); ++i) {
			Rect row = GetWorldRect();
			row.position.y += static_cast<float>(i) * itemHeight;
			row.size.y = itemHeight;

			if (static_cast<int>(i) == m_selectedIndex) {
				UIDrawCommand sel;
				sel.type = UIDrawType::Rect;
				sel.rect = row;
				sel.color = glm::vec4(0.2f, 0.44f, 0.72f, 0.85f);
				sel.layer = GetLayer() + 1;
				renderer.Submit(sel);
			}

			UIDrawCommand txt;
			txt.type = UIDrawType::Text;
			txt.rect = row;
			txt.text = m_items[i];
			txt.color = glm::vec4(0.95f, 0.95f, 0.95f, 1.0f);
			txt.layer = GetLayer() + 2;
			renderer.Submit(txt);
		}
	}

private:
	std::vector<std::string> m_items;
	int m_selectedIndex = -1;
};

} // namespace UI
} // namespace Runtime
