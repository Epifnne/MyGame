#pragma once

#include <string>
#include <utility>
#include <vector>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class DropDown : public Widget {
public:
	explicit DropDown(std::string id = {}) : Widget(std::move(id), "DropDown") {}

	void SetOptions(std::vector<std::string> options) { m_options = std::move(options); }
	const std::vector<std::string>& GetOptions() const { return m_options; }

	int GetSelectedIndex() const { return m_selectedIndex; }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (e.type != UIEventType::PointerDown || !GetWorldRect().Contains(e.screenPosition)) {
			return false;
		}

		if (!m_expanded) {
			m_expanded = true;
			return true;
		}

		if (!m_options.empty()) {
			const float itemHeight = 22.0f;
			const float localY = e.screenPosition.y - GetWorldRect().position.y;
			const int index = static_cast<int>((localY - itemHeight) / itemHeight);
			if (index >= 0 && index < static_cast<int>(m_options.size())) {
				m_selectedIndex = index;
			}
		}
		m_expanded = false;
		return true;
	}

	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand head;
		head.type = UIDrawType::Rect;
		head.rect = GetWorldRect();
		head.rect.size.y = 22.0f;
		head.color = glm::vec4(0.16f, 0.16f, 0.2f, 1.0f);
		head.layer = GetLayer();
		renderer.Submit(head);

		UIDrawCommand headText;
		headText.type = UIDrawType::Text;
		headText.rect = head.rect;
		headText.text = CurrentLabel();
		headText.color = glm::vec4(0.95f, 0.95f, 0.95f, 1.0f);
		headText.layer = GetLayer() + 1;
		renderer.Submit(headText);

		if (!m_expanded) {
			return;
		}

		for (size_t i = 0; i < m_options.size(); ++i) {
			Rect row = head.rect;
			row.position.y += 22.0f * static_cast<float>(i + 1);

			UIDrawCommand bg;
			bg.type = UIDrawType::Rect;
			bg.rect = row;
			bg.color = (static_cast<int>(i) == m_selectedIndex) ? glm::vec4(0.2f, 0.44f, 0.72f, 0.9f)
																 : glm::vec4(0.12f, 0.12f, 0.16f, 0.95f);
			bg.layer = GetLayer();
			renderer.Submit(bg);

			UIDrawCommand txt;
			txt.type = UIDrawType::Text;
			txt.rect = row;
			txt.text = m_options[i];
			txt.color = glm::vec4(0.95f, 0.95f, 0.95f, 1.0f);
			txt.layer = GetLayer() + 1;
			renderer.Submit(txt);
		}
	}

private:
	std::string CurrentLabel() const {
		if (m_selectedIndex >= 0 && m_selectedIndex < static_cast<int>(m_options.size())) {
			return m_options[static_cast<size_t>(m_selectedIndex)];
		}
		return m_options.empty() ? std::string("Select") : m_options.front();
	}

	std::vector<std::string> m_options;
	int m_selectedIndex = -1;
	bool m_expanded = false;
};

} // namespace UI
} // namespace Runtime
