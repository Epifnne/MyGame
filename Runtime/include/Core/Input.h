// Engine/include/Core/Input.h

#pragma once

#include<cstdint>

namespace Runtime {
namespace Core {

enum KeyCode : uint16_t {
    // printable keys
    Key_Unknown = 0,
    Key_Space = 32,
    Key_Apostrophe = 39,
    Key_Comma = 44,
    Key_Minus = 45,
    Key_Period = 46,
    Key_Slash = 47,
    Key_0 = 48, Key_1, Key_2, Key_3, Key_4, Key_5, Key_6, Key_7, Key_8, Key_9,
    Key_Semicolon = 59,
    Key_Equal = 61,
    Key_A = 65, Key_B, Key_C, Key_D, Key_E, Key_F, Key_G, Key_H, Key_I, Key_J,
         Key_K, Key_L, Key_M, Key_N, Key_O, Key_P, Key_Q, Key_R, Key_S, Key_T,
         Key_U, Key_V, Key_W, Key_X, Key_Y, Key_Z,
    Key_LeftBracket = 91,
    Key_Backslash = 92,
    Key_RightBracket = 93,
    Key_GraveAccent = 96,
    
    // function keys
    Key_Escape = 256,
    Key_Enter = 257,
    Key_Tab = 258,
    Key_Backspace = 259,
    Key_Insert = 260,
    Key_Delete = 261,
    Key_Right = 262,
    Key_Left = 263,
    Key_Down = 264,
    Key_Up = 265,
    Key_PageUp = 266,
    Key_PageDown = 267,
    Key_Home = 268,
    Key_End = 269,
    Key_F1 = 290, Key_F2, Key_F3, Key_F4, Key_F5, Key_F6, Key_F7, Key_F8, Key_F9,
         Key_F10, Key_F11, Key_F12,
    
    // mouse buttons
    Mouse_Left = 340,
    Mouse_Right = 341,
    Mouse_Middle = 342,
    Mouse_Button4 = 343,
    Mouse_Button5 = 344,
    
    Key_Count = 512
};

enum KeyStateFlags : uint8_t {
    State_None = 0,
    State_Down = 1 << 0,
    State_Pressed = 1 << 1,
    State_Released = 1 << 2,
};

class Input {
public:
    static Input& Get() {
        static Input instance;
        return instance;
    }
    
    // frame update - should be called at the start of each frame to reset transient states
    void BeginFrame() {
        for (int i = 0; i < Key_Count; ++i) {
            m_states[i] &= ~(State_Pressed | State_Released);
        }
        m_scrollDelta = 0.0f;
    }
    
    // key state updates - called by platform layer
    void SetKeyDown(KeyCode key) {
        if (!(m_states[key] & State_Down)) {
            m_states[key] |= State_Down | State_Pressed;
        }
    }
    
    void SetKeyUp(KeyCode key) {
        if (m_states[key] & State_Down) {
            m_states[key] = (m_states[key] & ~State_Down) | State_Released;
        }
    }
    
    // key state queries
    bool IsKeyDown(KeyCode key) const {
        return (m_states[key] & State_Down) != 0;
    }
    
    bool IsKeyPressed(KeyCode key) const {
        return (m_states[key] & State_Pressed) != 0;
    }
    
    bool IsKeyReleased(KeyCode key) const {
        return (m_states[key] & State_Released) != 0;
    }
    
    // mouse position
    void SetMousePosition(double x, double y) {
        m_mouseX = x;
        m_mouseY = y;
    }
    
    double GetMouseX() const { return m_mouseX; }
    double GetMouseY() const { return m_mouseY; }
    double GetMouseDeltaX() const { return m_mouseX - m_lastMouseX; }
    double GetMouseDeltaY() const { return m_mouseY - m_lastMouseY; }
    
    void UpdateMouseDelta() {
        m_lastMouseX = m_mouseX;
        m_lastMouseY = m_mouseY;
    }
    
    // scroll wheel
    void SetScrollDelta(float delta) { m_scrollDelta = delta; }
    float GetScrollDelta() const { return m_scrollDelta; }
    
private:
    Input() {
        for (int i = 0; i < Key_Count; ++i) {
            m_states[i] = State_None;
        }
    }
    
    uint8_t m_states[Key_Count];
    double m_mouseX = 0.0, m_mouseY = 0.0;
    double m_lastMouseX = 0.0, m_lastMouseY = 0.0;
    float m_scrollDelta = 0.0f;
};

}// namespace Core
}// namespace Runtime