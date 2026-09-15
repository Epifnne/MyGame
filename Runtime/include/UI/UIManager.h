#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "UI/Canvas.h"
#include "UI/DataBinding.h"
#include "UI/DebugUI.h"
#include "UI/Layout.h"
#include "UI/UIEvent.h"
#include "UI/UIRenderer.h"
#include "UI/UIStyle.h"

namespace Runtime {
namespace ECS {
class EventBus;
}

namespace UI {

class UIManager {
public:
	UIManager() = default;

	bool Initialize(Runtime::ECS::EventBus* eventBus = nullptr) {
		m_eventBus = eventBus;
		m_canvases.clear();
		m_renderer = std::make_unique<UIRenderer>();
		m_debugUI.Initialize();
		return true;
	}

	void Shutdown() {
		m_canvases.clear();
		m_debugUI.Shutdown();
		m_renderer.reset();
		m_eventBus = nullptr;
	}

	[[nodiscard("created canvas must be retained or retrieved by id")]] std::shared_ptr<Canvas> CreateCanvas(const std::string& id, CanvasSpace space, int sortOrder) {
		auto canvas = std::make_shared<Canvas>(id, space, sortOrder);
		m_canvases.push_back(canvas);
		SortCanvases();
		return canvas;
	}

	[[nodiscard]] const std::vector<std::shared_ptr<Canvas>>& GetCanvases() const { return m_canvases; }

	[[nodiscard]] std::shared_ptr<Canvas> FindCanvas(const std::string& id) const {
		for (const auto& canvas : m_canvases) {
			if (canvas && canvas->GetId() == id) {
				return canvas;
			}
		}
		return nullptr;
	}

	void Update(float dt, const Rect& viewportRect) {
		for (const auto& canvas : m_canvases) {
			if (!canvas || !canvas->IsVisible()) {
				continue;
			}
			LayoutEngine::LayoutTree(*canvas, viewportRect);
			canvas->Update(dt);
		}
	}

	void Render() {
		if (!m_renderer) {
			return;
		}

		m_renderer->BeginFrame();
		for (const auto& canvas : m_canvases) {
			if (canvas) {
				canvas->CollectDrawCommands(*m_renderer);
			}
		}
		m_renderer->SortByLayer();

		m_debugUI.BeginFrame();
		m_debugUI.PushLine("UIManager: canvases=" + std::to_string(m_canvases.size()));
		m_debugUI.PushLine("UIRenderer: commands=" + std::to_string(m_renderer->GetStats().commandCount));
		(void)m_debugUI.EndFrame();
	}

	bool RouteEvent(const UIEvent& event) {
		UIEvent routed = event;
		for (auto it = m_canvases.rbegin(); it != m_canvases.rend(); ++it) {
			if (!(*it) || !(*it)->IsVisible()) {
				continue;
			}

			Widget* target = (*it)->FindTopWidgetAt(event.screenPosition);
			if (!target) {
				continue;
			}

			routed.target = target;
			const bool consumed = target->DispatchEvent(routed);
			if (consumed) {
				return true;
			}
		}
		return false;
	}

	[[nodiscard]] UIRenderer* GetRenderer() { return m_renderer.get(); }
	[[nodiscard]] const UIRenderer* GetRenderer() const { return m_renderer.get(); }

	[[nodiscard]] DataBindingContext& Bindings() { return m_bindings; }
	[[nodiscard]] UIStyleSheet& Styles() { return m_styles; }
	[[nodiscard]] DebugUI& DebugOverlay() { return m_debugUI; }

private:
	void SortCanvases() {
		std::stable_sort(m_canvases.begin(), m_canvases.end(), [](const std::shared_ptr<Canvas>& a, const std::shared_ptr<Canvas>& b) {
			if (!a || !b) {
				return static_cast<bool>(a);
			}
			return a->GetSortOrder() < b->GetSortOrder();
		});
	}

	Runtime::ECS::EventBus* m_eventBus = nullptr;
	std::vector<std::shared_ptr<Canvas>> m_canvases;
	std::unique_ptr<UIRenderer> m_renderer;
	DataBindingContext m_bindings;
	UIStyleSheet m_styles;
	DebugUI m_debugUI;
};

} // namespace UI
} // namespace Runtime
