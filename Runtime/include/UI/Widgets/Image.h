#pragma once

#include <string>

#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

#include "UI/UIRenderer.h"

namespace Runtime {
namespace UI {

class Image : public Widget {
public:
	explicit Image(std::string id = {}) : Widget(std::move(id), "Image") {}

	void SetTextureId(uint32_t textureId) { m_textureId = textureId; }
	uint32_t GetTextureId() const { return m_textureId; }

	void SetTint(const glm::vec4& tint) { m_tint = tint; }

protected:
	void OnRender(UIRenderer& renderer) const override {
		UIDrawCommand cmd;
		cmd.type = UIDrawType::Image;
		cmd.rect = GetWorldRect();
		cmd.textureId = m_textureId;
		cmd.color = m_tint;
		cmd.layer = GetLayer();
		renderer.Submit(cmd);
	}

private:
	uint32_t m_textureId = 0;
	glm::vec4 m_tint{1.0f, 1.0f, 1.0f, 1.0f};
};

} // namespace UI
} // namespace Runtime
