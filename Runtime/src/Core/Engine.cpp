#include "Core/Engine.h"
#include "Core/Input.h"
#include "Platform/Window.h"
#include "Graphics/Renderer.h"
#include <iostream>

namespace Runtime {
namespace Core {

bool Engine::Initialize(int width, int height, const char* title) {
    if (m_isInitialized) {
        std::cerr << "Engine is already initialized." << std::endl;
        return false;
    }

    if (!InitializeSubsystems(width, height, title)) {
        std::cerr << "Failed to initialize engine subsystems." << std::endl;
        return false;
    }

    m_isInitialized = true;
    m_isRunning = true;

    if (m_gameInit) {
        m_gameInit();
    }

    return true;
}

void Engine::Shutdown() {
    if (!m_isInitialized) {
        return;
    }

    m_isRunning = false;

    if (m_gameShutdown) {
        m_gameShutdown();
    }

    CleanupSubsystems();
    m_isInitialized = false;
}

bool Engine::InitializeSubsystems(int width, int height, const char* title) {
    m_width = width;
    m_height = height;

    m_window = new Platform::Window();
    if (!m_window->Initialize(width, height, title, true)) {
        delete m_window;
        m_window = nullptr;
        return false;
    }

    m_renderer = new Graphics::Renderer();
    if (!m_renderer->Initialize()) {
        delete m_window;
        m_window = nullptr;
        delete m_renderer;
        m_renderer = nullptr;
        return false;
    }

    m_renderer->SetViewport(0, 0, width, height);

    return true;
}

void Engine::CleanupSubsystems() {
    if (m_renderer) {
        m_renderer->Shutdown();
        delete m_renderer;
        m_renderer = nullptr;
    }

    if (m_window) {
        m_window->Shutdown();
        delete m_window;
        m_window = nullptr;
    }
}

void Engine::FixedUpdate(float deltatime) {
    // 引擎层的固定更新点，提供给 GameLoop 回调使用。
    (void)deltatime; // 当前用例无动作，供后续扩展（物理/定时逻辑）
}

void Engine::ProcessWindowEvents() {
    if (m_window) {
        m_window->PollEvents();
        if (m_window->ShouldClose()) {
            m_isRunning = false;
        }
    }
}

} // namespace Core
} // namespace Runtime
