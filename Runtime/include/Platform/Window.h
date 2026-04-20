#pragma once

#include <string>
#include <GLFW/glfw3.h>

namespace Runtime {
namespace Platform {

class Window {
public:
    Window() = default;
    ~Window();

    bool Initialize(int width, int height, const char* title, bool vsync = true);
    void Shutdown();

    void PollEvents();
    void SwapBuffers();

    bool ShouldClose() const;
    void SetShouldClose(bool value);

    void SetVSync(bool enabled);

    int GetWidth() const;
    int GetHeight() const;
    GLFWwindow* GetNativeWindow() const;

private:
    bool m_initialized = false;
    GLFWwindow* m_window = nullptr;
    int m_width = 0;
    int m_height = 0;
};

} // namespace Platform
} // namespace Runtime
