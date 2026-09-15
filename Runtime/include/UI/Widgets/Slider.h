#pragma once

#include <algorithm>
#include <functional>
#include <string>

#include <glm/vec4.hpp>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

enum class SliderDirection {
	Horizontal,
	Vertical
};

class Slider : public Widget {
public:
	explicit Slider(std::string id = {}) : Widget(std::move(id), "Slider") {}

	void SetRange(float minValue, float maxValue) {
		m_min = minValue;
		m_max = std::max(minValue, maxValue);
		SetValue(m_value);
	}

	void SetValue(float value) {
		m_value = std::clamp(value, m_min, m_max);
		if (m_onValueChanged) {
			m_onValueChanged(m_value);
		}
	}

	float GetValue() const { return m_value; }
	void SetDirection(SliderDirection direction) { m_direction = direction; }
	void SetOnValueChanged(std::function<void(float)> onValueChanged) { m_onValueChanged = std::move(onValueChanged); }

protected:
	bool OnEvent(const UIEvent& e) override {
		const Rect rect = GetWorldRect();
		if (e.type == UIEventType::PointerDown && rect.Contains(e.screenPosition)) {
			m_dragging = true;
			UpdateValueFromPoint(e.screenPosition);
			return true;
		}

		if (e.type == UIEventType::PointerMove && m_dragging) {
			UpdateValueFromPoint(e.screenPosition);
			return true;
		}

		if (e.type == UIEventType::PointerUp) {
			m_dragging = false;
		}
		return false;
	}

	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand track;
		track.type = UIDrawType::Rect;
		track.rect = GetWorldRect();
		track.color = glm::vec4(0.2f, 0.2f, 0.24f, 1.0f);
		track.layer = GetLayer();
		renderer.Submit(track);

		const float t = (m_max > m_min) ? ((m_value - m_min) / (m_max - m_min)) : 0.0f;
		Rect fillRect = GetWorldRect();
		if (m_direction == SliderDirection::Horizontal) {
			fillRect.size.x = fillRect.size.x * std::clamp(t, 0.0f, 1.0f);
		} else {
			fillRect.size.y = fillRect.size.y * std::clamp(t, 0.0f, 1.0f);
		}

		UIDrawCommand fill;
		fill.type = UIDrawType::Rect;
		fill.rect = fillRect;
		fill.color = glm::vec4(0.18f, 0.52f, 0.88f, 1.0f);
		fill.layer = GetLayer() + 1;
		renderer.Submit(fill);
	}

private:
	void UpdateValueFromPoint(const glm::vec2& p) {
		const Rect rect = GetWorldRect();
		float t = 0.0f;
		if (m_direction == SliderDirection::Horizontal) {
			t = (rect.size.x <= 0.0f) ? 0.0f : ((p.x - rect.position.x) / rect.size.x);
		} else {
			t = (rect.size.y <= 0.0f) ? 0.0f : ((p.y - rect.position.y) / rect.size.y);
		}
		SetValue(m_min + (m_max - m_min) * std::clamp(t, 0.0f, 1.0f));
	}

	float m_min = 0.0f;
	float m_max = 1.0f;
	float m_value = 0.0f;
	bool m_dragging = false;
	SliderDirection m_direction = SliderDirection::Horizontal;
	std::function<void(float)> m_onValueChanged;
};

} // namespace UI
} // namespace Runtime
