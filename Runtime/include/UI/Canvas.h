#pragma once

#include <string>

#include "UI/Widget.h"

namespace Runtime {
namespace UI {

enum class CanvasSpace {
	Screen,
	World
};

class Canvas final : public Widget {
public:
	Canvas(std::string id, CanvasSpace space = CanvasSpace::Screen, int sortOrder = 0)
		: Widget(std::move(id), "Canvas"), m_space(space), m_sortOrder(sortOrder) {}

	CanvasSpace GetSpace() const { return m_space; }
	int GetSortOrder() const { return m_sortOrder; }

	void SetSpace(CanvasSpace space) { m_space = space; }
	void SetSortOrder(int order) { m_sortOrder = order; }

private:
	CanvasSpace m_space = CanvasSpace::Screen;
	int m_sortOrder = 0;
};

} // namespace UI
} // namespace Runtime
