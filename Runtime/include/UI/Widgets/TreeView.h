#pragma once

#include <string>
#include <utility>
#include <vector>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

struct TreeNode {
	std::string label;
	bool expanded = true;
	std::vector<TreeNode> children;
};

class TreeView : public Widget {
public:
	explicit TreeView(std::string id = {}) : Widget(std::move(id), "TreeView") {}

	std::vector<TreeNode>& Nodes() { return m_nodes; }
	const std::vector<TreeNode>& Nodes() const { return m_nodes; }

protected:
	bool OnEvent(const UIEvent& e) override {
		if (e.type != UIEventType::PointerDown || !GetWorldRect().Contains(e.screenPosition)) {
			return false;
		}
		if (!m_nodes.empty()) {
			m_nodes.front().expanded = !m_nodes.front().expanded;
		}
		return true;
	}

	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand bg;
		bg.type = UIDrawType::Rect;
		bg.rect = GetWorldRect();
		bg.color = glm::vec4(0.1f, 0.1f, 0.12f, 0.95f);
		bg.layer = GetLayer();
		renderer.Submit(bg);

		float y = GetWorldRect().position.y;
		RenderNodes(renderer, m_nodes, 0, y);
	}

private:
	void RenderNodes(UIRenderer& renderer, const std::vector<TreeNode>& nodes, int depth, float& y) const {
		for (const TreeNode& node : nodes) {
			Rect row;
			row.position = glm::vec2(GetWorldRect().position.x + static_cast<float>(depth) * 14.0f, y);
			row.size = glm::vec2(GetWorldRect().size.x, 18.0f);
			y += 18.0f;

			UIDrawCommand txt;
			txt.type = UIDrawType::Text;
			txt.rect = row;
			txt.text = (node.children.empty() ? "  " : (node.expanded ? "- " : "+ ")) + node.label;
			txt.color = glm::vec4(0.9f, 0.9f, 0.9f, 1.0f);
			txt.layer = GetLayer() + 1;
			renderer.Submit(txt);

			if (node.expanded && !node.children.empty()) {
				RenderNodes(renderer, node.children, depth + 1, y);
			}
		}
	}

	std::vector<TreeNode> m_nodes;
};

} // namespace UI
} // namespace Runtime
