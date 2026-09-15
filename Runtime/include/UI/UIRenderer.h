#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "UI/Widget.h"

namespace Runtime {
namespace UI {

enum class UIDrawType {
	Rect,
	Text,
	Image
};

enum class UITextRenderTechnique {
	Bitmap,
	SDF,
	MSDF
};

enum class UITextUsage {
	Overlay,
	WorldFloating
};

struct UIDrawCommand {
	UIDrawType type = UIDrawType::Rect;
	Rect rect;
	glm::vec4 color{1.0f, 1.0f, 1.0f, 1.0f};
	std::string text;
	uint32_t textureId = 0;
	int layer = 0;
	UITextRenderTechnique textTechnique = UITextRenderTechnique::Bitmap;
	UITextUsage textUsage = UITextUsage::Overlay;
	glm::vec3 worldTextPosition{0.0f, 0.0f, 0.0f};
};

struct UIRenderStats {
	size_t commandCount = 0;
	size_t rectCount = 0;
	size_t textCount = 0;
	size_t imageCount = 0;
};

class UIRenderer {
public:
	void BeginFrame() {
		m_commands.clear();
		m_stats = {};
	}

	void Submit(const UIDrawCommand& cmd) {
		m_commands.push_back(cmd);
		++m_stats.commandCount;
		if (cmd.type == UIDrawType::Rect) {
			++m_stats.rectCount;
		} else if (cmd.type == UIDrawType::Text) {
			++m_stats.textCount;
		} else if (cmd.type == UIDrawType::Image) {
			++m_stats.imageCount;
		}
	}

	void SortByLayer() {
		std::stable_sort(m_commands.begin(), m_commands.end(), [](const UIDrawCommand& a, const UIDrawCommand& b) {
			return a.layer < b.layer;
		});
	}

	const std::vector<UIDrawCommand>& GetCommands() const { return m_commands; }
	const UIRenderStats& GetStats() const { return m_stats; }

private:
	std::vector<UIDrawCommand> m_commands;
	UIRenderStats m_stats;
};

} // namespace UI
} // namespace Runtime
