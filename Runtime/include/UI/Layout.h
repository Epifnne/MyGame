#pragma once

#include <algorithm>

#include "UI/Widget.h"

namespace Runtime {
namespace UI {

class LayoutEngine {
public:
	static void ApplyAnchoredLayout(Widget& widget, const Rect& parentRect) {
		const glm::vec2 anchorMin = widget.GetAnchorMin();
		const glm::vec2 anchorMax = widget.GetAnchorMax();
		const glm::vec2 marginMin = widget.GetMarginMin();
		const glm::vec2 marginMax = widget.GetMarginMax();

		const glm::vec2 anchorPosMin = parentRect.position + parentRect.size * anchorMin + marginMin;
		const glm::vec2 anchorPosMax = parentRect.position + parentRect.size * anchorMax - marginMax;

		glm::vec2 size = widget.GetSize();
		glm::vec2 pos = widget.GetPosition();

		const bool stretchX = std::abs(anchorMax.x - anchorMin.x) > 1e-4f;
		const bool stretchY = std::abs(anchorMax.y - anchorMin.y) > 1e-4f;

		if (stretchX) {
			size.x = std::max(0.0f, anchorPosMax.x - anchorPosMin.x);
			pos.x = anchorPosMin.x - parentRect.position.x;
		}

		if (stretchY) {
			size.y = std::max(0.0f, anchorPosMax.y - anchorPosMin.y);
			pos.y = anchorPosMin.y - parentRect.position.y;
		}

		widget.SetPosition(pos);
		widget.SetSize(size);
	}

	static void LayoutTree(Widget& root, const Rect& parentRect) {
		ApplyAnchoredLayout(root, parentRect);
		const Rect rootRect = root.GetWorldRect();
		for (const auto& child : root.GetChildren()) {
			if (child) {
				LayoutTree(*child, rootRect);
			}
		}
	}
};

} // namespace UI
} // namespace Runtime
