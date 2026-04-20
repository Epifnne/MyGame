#include "Platform/Window.h"
#include <iostream>

namespace Runtime {
namespace Platform {

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
