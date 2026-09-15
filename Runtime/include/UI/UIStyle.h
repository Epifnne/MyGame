#pragma once

#include <string>
#include <unordered_map>

#include <glm/vec4.hpp>

namespace Runtime {
namespace UI {

struct UIStyle {
	glm::vec4 background{0.12f, 0.12f, 0.14f, 0.9f};
	glm::vec4 foreground{0.92f, 0.92f, 0.92f, 1.0f};
	glm::vec4 border{0.0f, 0.0f, 0.0f, 0.25f};
	glm::vec4 accent{0.24f, 0.56f, 0.94f, 1.0f};
	float borderWidth = 1.0f;
	float cornerRadius = 2.0f;
};

class UIStyleSheet {
public:
	void SetGlobalStyle(const UIStyle& style) { m_globalStyle = style; }
	[[nodiscard]] const UIStyle& GetGlobalStyle() const { return m_globalStyle; }

	void SetStyleForType(const std::string& typeName, const UIStyle& style) { m_stylesByType[typeName] = style; }

	[[nodiscard]] const UIStyle& Resolve(const std::string& typeName) const {
		const auto it = m_stylesByType.find(typeName);
		if (it != m_stylesByType.end()) {
			return it->second;
		}
		return m_globalStyle;
	}

private:
	UIStyle m_globalStyle;
	std::unordered_map<std::string, UIStyle> m_stylesByType;
};

} // namespace UI
} // namespace Runtime
