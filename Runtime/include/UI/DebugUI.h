#pragma once

#include <string>
#include <vector>

namespace Runtime {
namespace UI {

class UIRenderer;

class DebugUI {
public:
	bool Initialize() {
		m_initialized = true;
		return true;
	}

	void Shutdown() {
		m_initialized = false;
		m_lines.clear();
	}

	void SetEnabled(bool enabled) { m_enabled = enabled; }
	bool IsEnabled() const { return m_enabled && m_initialized; }

	void BeginFrame() {
		if (!IsEnabled()) {
			return;
		}
		m_lines.clear();
	}

	void PushLine(std::string line) {
		if (!IsEnabled()) {
			return;
		}
		m_lines.push_back(std::move(line));
	}

	const std::vector<std::string>& EndFrame() const { return m_lines; }

private:
	bool m_initialized = false;
	bool m_enabled = true;
	std::vector<std::string> m_lines;
};

} // namespace UI
} // namespace Runtime
