#include "Platform/Window.h"
#include "Core/Input.h"
#include <iostream>

namespace Runtime {
namespace Platform {

namespace {

Runtime::Core::KeyCode ToRuntimeMouseButton(int glfwButton) {
    switch (glfwButton) {
    case GLFW_MOUSE_BUTTON_LEFT:
        return Runtime::Core::Mouse_Left;
    case GLFW_MOUSE_BUTTON_RIGHT:
        return Runtime::Core::Mouse_Right;
    case GLFW_MOUSE_BUTTON_MIDDLE:
        return Runtime::Core::Mouse_Middle;
    case GLFW_MOUSE_BUTTON_4:
        return Runtime::Core::Mouse_Button4;
    case GLFW_MOUSE_BUTTON_5:
        return Runtime::Core::Mouse_Button5;
    default:
        return Runtime::Core::Key_Unknown;
    }
}

} // namespace

static void GLFWErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error [" << error << "] : " << description << std::endl;
}

Window::~Window() {
    Shutdown();
}

bool Window::Initialize(int width, int height, const char* title, bool vsync) {
    if (m_initialized) {
        return true;
    }

    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    glfwSetErrorCallback(GLFWErrorCallback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    m_window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!m_window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    SetVSync(vsync);

    m_width = width;
    m_height = height;

    auto* input = &Runtime::Core::Input::Get();
    glfwSetWindowUserPointer(m_window, input);

    glfwSetKeyCallback(m_window, [](GLFWwindow* window, int key, int, int action, int) {
        auto* runtimeInput = static_cast<Runtime::Core::Input*>(glfwGetWindowUserPointer(window));
        if (!runtimeInput || key < 0 || key >= Runtime::Core::Key_Count) {
            return;
        }

        const auto runtimeKey = static_cast<Runtime::Core::KeyCode>(key);
        if (action == GLFW_PRESS || action == GLFW_REPEAT) {
            runtimeInput->SetKeyDown(runtimeKey);
        } else if (action == GLFW_RELEASE) {
            runtimeInput->SetKeyUp(runtimeKey);
        }
    });

    glfwSetMouseButtonCallback(m_window, [](GLFWwindow* window, int button, int action, int) {
        auto* runtimeInput = static_cast<Runtime::Core::Input*>(glfwGetWindowUserPointer(window));
        if (!runtimeInput) {
            return;
        }

        const auto runtimeButton = ToRuntimeMouseButton(button);
        if (runtimeButton == Runtime::Core::Key_Unknown) {
            return;
        }

        if (action == GLFW_PRESS) {
            runtimeInput->SetKeyDown(runtimeButton);
        } else if (action == GLFW_RELEASE) {
            runtimeInput->SetKeyUp(runtimeButton);
        }
    });

    glfwSetCursorPosCallback(m_window, [](GLFWwindow* window, double x, double y) {
        auto* runtimeInput = static_cast<Runtime::Core::Input*>(glfwGetWindowUserPointer(window));
        if (runtimeInput) {
            runtimeInput->SetMousePosition(x, y);
        }
    });

    glfwSetScrollCallback(m_window, [](GLFWwindow* window, double, double yoffset) {
        auto* runtimeInput = static_cast<Runtime::Core::Input*>(glfwGetWindowUserPointer(window));
        if (runtimeInput) {
            runtimeInput->SetScrollDelta(static_cast<float>(yoffset));
        }
    });

    m_initialized = true;

    return true;
}

void Window::Shutdown() {
    if (!m_initialized) {
        return;
    }

    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
    m_initialized = false;
}

void Window::PollEvents() {
    if (m_window) {
        glfwPollEvents();
    }
}

void Window::SwapBuffers() {
    if (m_window) {
        glfwSwapBuffers(m_window);
    }
}

bool Window::ShouldClose() const {
    return m_window ? glfwWindowShouldClose(m_window) : true;
}

void Window::SetShouldClose(bool value) {
    if (m_window) {
        glfwSetWindowShouldClose(m_window, value ? GLFW_TRUE : GLFW_FALSE);
    }
}

void Window::SetVSync(bool enabled) {
    glfwSwapInterval(enabled ? 1 : 0);
}

int Window::GetWidth() const {
    return m_width;
}

int Window::GetHeight() const {
    return m_height;
}

GLFWwindow* Window::GetNativeWindow() const {
    return m_window;
}

} // namespace Platform
} // namespace Runtime
