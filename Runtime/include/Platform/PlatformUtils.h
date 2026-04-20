#pragma once

namespace Runtime {
namespace Platform {

inline void EnableVSync(bool enabled) {
    (void)enabled;
    // 该函数在 Window::SetVSync 已经实现，可根据平台扩展。
}

} // namespace Platform
} // namespace Runtime
