#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glm/vec2.hpp>

#include "UI/UIEvent.h"

namespace Runtime {
namespace UI {

class UIRenderer;

struct Rect {
	glm::vec2 position{0.0f, 0.0f};
	glm::vec2 size{0.0f, 0.0f};

	[[nodiscard]] bool Contains(const glm::vec2& p) const {
		return p.x >= position.x && p.y >= position.y && p.x <= position.x + size.x && p.y <= position.y + size.y;
	}
};

class Widget : public std::enable_shared_from_this<Widget> {
public:
	Widget(std::string id = {}, std::string typeName = "Widget")
		: m_id(std::move(id)), m_typeName(std::move(typeName)) {}
	virtual ~Widget() = default;

	[[nodiscard]] const std::string& GetId() const { return m_id; }
	[[nodiscard]] const std::string& GetTypeName() const { return m_typeName; }

	void SetPosition(const glm::vec2& p) { m_position = p; }
	void SetSize(const glm::vec2& s) { m_size = s; }
	void SetVisible(bool v) { m_visible = v; }
	void SetEnabled(bool e) { m_enabled = e; }
	void SetLayer(int layer) { m_layer = layer; }
	void SetAnchor(const glm::vec2& minAnchor, const glm::vec2& maxAnchor) {
		m_anchorMin = minAnchor;
		m_anchorMax = maxAnchor;
	}
	void SetPivot(const glm::vec2& pivot) { m_pivot = pivot; }
	void SetMargins(const glm::vec2& minMargin, const glm::vec2& maxMargin) {
		m_marginMin = minMargin;
		m_marginMax = maxMargin;
	}

	const glm::vec2& GetPosition() const { return m_position; }
	const glm::vec2& GetSize() const { return m_size; }
	const glm::vec2& GetAnchorMin() const { return m_anchorMin; }
	const glm::vec2& GetAnchorMax() const { return m_anchorMax; }
	const glm::vec2& GetPivot() const { return m_pivot; }
	const glm::vec2& GetMarginMin() const { return m_marginMin; }
	const glm::vec2& GetMarginMax() const { return m_marginMax; }
	[[nodiscard]] bool IsVisible() const { return m_visible; }
	[[nodiscard]] bool IsEnabled() const { return m_enabled; }
	int GetLayer() const { return m_layer; }

	[[nodiscard]] Widget* GetParent() const { return m_parent; }
	[[nodiscard]] const std::vector<std::shared_ptr<Widget>>& GetChildren() const { return m_children; }

	std::shared_ptr<Widget> AddChild(std::shared_ptr<Widget> child) {
		if (!child || child.get() == this) {
			return nullptr;
		}
		child->m_parent = this;
		m_children.push_back(child);
		return child;
	}

	bool RemoveChild(const Widget* child) {
		const auto it = std::find_if(m_children.begin(), m_children.end(), [child](const std::shared_ptr<Widget>& p) {
			return p.get() == child;
		});
		if (it == m_children.end()) {
			return false;
		}
		(*it)->m_parent = nullptr;
		m_children.erase(it);
		return true;
	}

	Rect GetLocalRect() const { return Rect{m_position, m_size}; }

	[[nodiscard]] glm::vec2 GetWorldPosition() const {
		const Widget* current = this;
		glm::vec2 world = current->m_position;
		current = current->m_parent;
		while (current) {
			world += current->m_position;
			current = current->m_parent;
		}
		return world;
	}

	Rect GetWorldRect() const { return Rect{GetWorldPosition(), m_size}; }

	virtual void Update(float dt) {
		OnUpdate(dt);
		for (const auto& child : m_children) {
			if (child && child->IsVisible()) {
				child->Update(dt);
			}
		}
	}

	bool DispatchEvent(const UIEvent& e) {
		if (!m_visible || !m_enabled) {
			return false;
		}

		for (auto it = m_children.rbegin(); it != m_children.rend(); ++it) {
			if (*it && (*it)->DispatchEvent(e)) {
				return true;
			}
		}

		return OnEvent(e);
	}

	void CollectDrawCommands(UIRenderer& renderer) const {
		if (!m_visible) {
			return;
		}
		OnRender(renderer);
		for (const auto& child : m_children) {
			if (child) {
				child->CollectDrawCommands(renderer);
			}
		}
	}

	[[nodiscard]] Widget* FindTopWidgetAt(const glm::vec2& p) {
		if (!m_visible) {
			return nullptr;
		}
		for (auto it = m_children.rbegin(); it != m_children.rend(); ++it) {
			if (!(*it)) {
				continue;
			}
			Widget* hit = (*it)->FindTopWidgetAt(p);
			if (hit) {
				return hit;
			}
		}
		return GetWorldRect().Contains(p) ? this : nullptr;
	}

protected:
	virtual void OnUpdate(float) {}
	virtual bool OnEvent(const UIEvent&) { return false; }
	virtual void OnRender(UIRenderer&) const {}

private:
	std::string m_id;
	std::string m_typeName;
	Widget* m_parent = nullptr;
	std::vector<std::shared_ptr<Widget>> m_children;

	glm::vec2 m_position{0.0f, 0.0f};
	glm::vec2 m_size{100.0f, 30.0f};
	glm::vec2 m_anchorMin{0.0f, 0.0f};
	glm::vec2 m_anchorMax{0.0f, 0.0f};
	glm::vec2 m_pivot{0.0f, 0.0f};
	glm::vec2 m_marginMin{0.0f, 0.0f};
	glm::vec2 m_marginMax{0.0f, 0.0f};

	bool m_visible = true;
	bool m_enabled = true;
	int m_layer = 0;
};

} // namespace UI
} // namespace Runtime
