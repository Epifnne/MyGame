// Engine/include/Core/Input.h

#pragma once

#include<cstddef>
#include<cstdint>
#include<functional>
#include<utility>
#include<vector>

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

enum class InputEventType : uint8_t {
    KeyDown,
    KeyUp,
    MouseMove,
    Scroll
};

struct InputEvent {
    InputEventType type = InputEventType::MouseMove;
    KeyCode key = Key_Unknown;
    double mouseX = 0.0;
    double mouseY = 0.0;
    float scrollDelta = 0.0f;
};

class Input {
public:
    using EventHandler = std::function<void(const InputEvent&)>;
    using EventHandlerId = uint32_t;

    static Input& Get() {
        static Input instance;
        return instance;
    }
    
    // frame update - clear transient flags then consume buffered input from previous frame
    void BeginFrame() {
        for (int i = 0; i < Key_Count; ++i) {
            m_states[i] &= ~(State_Pressed | State_Released);
        }
        m_scrollDelta = 0.0f;

        m_dispatchHandlers = m_eventHandlers;

        for (const InputEvent& event : m_pendingEvents) {
            switch (event.type) {
            case InputEventType::KeyDown:
                if (!(m_states[event.key] & State_Down)) {
                    m_states[event.key] |= State_Down | State_Pressed;
                }
                break;
            case InputEventType::KeyUp:
                if (m_states[event.key] & State_Down) {
                    m_states[event.key] = (m_states[event.key] & ~State_Down) | State_Released;
                }
                break;
            case InputEventType::MouseMove:
                m_mouseX = event.mouseX;
                m_mouseY = event.mouseY;
                break;
            case InputEventType::Scroll:
                m_scrollDelta += event.scrollDelta;
                break;
            }

            for (const EventHandlerEntry& handler : m_dispatchHandlers) {
                if (handler.callback) {
                    handler.callback(event);
                }
            }
        }
        m_pendingEvents.clear();
    }
    
    // key state updates - called by platform layer
    void SetKeyDown(KeyCode key) {
        if (key <= Key_Unknown || key >= Key_Count) {
            return;
        }
        InputEvent event;
        event.type = InputEventType::KeyDown;
        event.key = key;
        m_pendingEvents.push_back(event);
    }
    
    void SetKeyUp(KeyCode key) {
        if (key <= Key_Unknown || key >= Key_Count) {
            return;
        }
        InputEvent event;
        event.type = InputEventType::KeyUp;
        event.key = key;
        m_pendingEvents.push_back(event);
    }

    EventHandlerId RegisterEventHandler(EventHandler handler) {
        if (!handler) {
            return 0;
        }

        EventHandlerId id = m_nextHandlerId++;
        m_eventHandlers.push_back(EventHandlerEntry{id, std::move(handler)});
        return id;
    }

    bool UnregisterEventHandler(EventHandlerId id) {
        if (id == 0) {
            return false;
        }

        for (size_t i = 0; i < m_eventHandlers.size(); ++i) {
            if (m_eventHandlers[i].id == id) {
                m_eventHandlers.erase(m_eventHandlers.begin() + static_cast<std::ptrdiff_t>(i));
                return true;
            }
        }
        return false;
    }

    void ClearEventHandlers() {
        m_eventHandlers.clear();
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
        InputEvent event;
        event.type = InputEventType::MouseMove;
        event.mouseX = x;
        event.mouseY = y;
        m_pendingEvents.push_back(event);
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
    void SetScrollDelta(float delta) {
        InputEvent event;
        event.type = InputEventType::Scroll;
        event.scrollDelta = delta;
        m_pendingEvents.push_back(event);
    }
    float GetScrollDelta() const { return m_scrollDelta; }
    
private:
    struct EventHandlerEntry {
        EventHandlerId id = 0;
        EventHandler callback;
    };

    Input() {
        for (int i = 0; i < Key_Count; ++i) {
            m_states[i] = State_None;
        }
        m_pendingEvents.reserve(128);
        m_eventHandlers.reserve(16);
        m_dispatchHandlers.reserve(16);
    }
    
    uint8_t m_states[Key_Count];
    double m_mouseX = 0.0, m_mouseY = 0.0;
    double m_lastMouseX = 0.0, m_lastMouseY = 0.0;
    float m_scrollDelta = 0.0f;
    std::vector<InputEvent> m_pendingEvents;
    std::vector<EventHandlerEntry> m_eventHandlers;
    std::vector<EventHandlerEntry> m_dispatchHandlers;
    EventHandlerId m_nextHandlerId = 1;
};

}// namespace Core
}// namespace Runtime