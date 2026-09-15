#pragma once

#include <functional>
#include <string>
#include <utility>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class Toggle : public Widget {
public:
	explicit Toggle(std::string id = {}) : Widget(std::move(id), "Toggle") {}

	void SetLabel(std::string label) { m_label = std::move(label); }
	void SetChecked(bool checked) { m_checked = checked; }
	bool IsChecked() const { return m_checked; }

	void SetOnChanged(std::function<void(bool)> onChanged) { m_onChanged = std::move(onChanged); }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (e.type == UIEventType::PointerDown && GetWorldRect().Contains(e.screenPosition)) {
			m_checked = !m_checked;
			if (m_onChanged) {
				m_onChanged(m_checked);
			}
			return true;
		}
		return false;
	}

	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand box;
		box.type = UIDrawType::Rect;
		box.rect = GetWorldRect();
		box.color = m_checked ? glm::vec4(0.18f, 0.6f, 0.34f, 1.0f) : glm::vec4(0.2f, 0.2f, 0.24f, 1.0f);
		box.layer = GetLayer();
		renderer.Submit(box);

		UIDrawCommand text;
		text.type = UIDrawType::Text;
		text.rect = GetWorldRect();
		text.text = m_label;
		text.color = glm::vec4(0.95f, 0.95f, 0.95f, 1.0f);
		text.layer = GetLayer() + 1;
		renderer.Submit(text);
	}

private:
	std::string m_label = "Toggle";
	bool m_checked = false;
	std::function<void(bool)> m_onChanged;
};

} // namespace UI
} // namespace Runtime
