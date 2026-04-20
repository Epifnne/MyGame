// Engine/include/Core/GameLoop.h
#pragma once

#include <chrono>
#include <type_traits>
#include <utility>

namespace Runtime {
namespace Core {

enum class GameLoopState : uint8_t {
    Running,
    Paused,
    Stopped
};

template<typename UpdateFn, typename RenderFn, typename FixedUpdateFn = void>
class GameLoop {
public:
    GameLoop(UpdateFn update, RenderFn render, FixedUpdateFn fixedUpdate = nullptr)
        : m_update(update)
        , m_render(render)
        , m_fixedUpdate(fixedUpdate)
        , m_fixedTimeStep(1.0f / 30.0f) // 30 FPS default
        , m_maxDeltaTime(0.033f) // 30 FPS limit
    {}
    
    void Start() {
        m_state = GameLoopState::Running;
        m_lastTime = std::chrono::high_resolution_clock::now();
        m_accumulator = 0.0f;
    }
    
    void Stop() { m_state = GameLoopState::Stopped; }
    void Pause() { if (m_state == GameLoopState::Running) m_state = GameLoopState::Paused; }
    void Resume() { if (m_state == GameLoopState::Paused) m_state = GameLoopState::Running; }
    
    GameLoopState GetState() const { return m_state; }
    
    void Tick() {
        if (m_state != GameLoopState::Running) return;
        
        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(currentTime - m_lastTime).count();
        m_lastTime = currentTime;
        
        if (deltaTime > m_maxDeltaTime) {
            deltaTime = m_maxDeltaTime;
        }
        
        m_update(deltaTime);
        
        if constexpr (!std::is_same_v<FixedUpdateFn, void>) {
            m_accumulator += deltaTime;
            while (m_accumulator >= m_fixedTimeStep) {
                m_fixedUpdate(m_fixedTimeStep);
                m_accumulator -= m_fixedTimeStep;
            }
        }
        
        m_render();
        m_frameCount++;
    }
    
    void SetFixedTimeStep(float step) { m_fixedTimeStep = step; }
    void SetMaxDeltaTime(float maxDt) { m_maxDeltaTime = maxDt; }
    int GetFrameCount() const { return m_frameCount; }
    
private:
    UpdateFn m_update;
    RenderFn m_render;
    FixedUpdateFn m_fixedUpdate;
    
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point m_lastTime;  // for delta time calculation
    
    GameLoopState m_state = GameLoopState::Stopped;
    float m_fixedTimeStep;  // fixed time step for fixed updates
    float m_maxDeltaTime; // to prevent spiral of death
    float m_accumulator = 0.0f; // for fixed update timing
    int m_frameCount = 0; // total frames since start
};

}// namespace Core
}// namespace Runtime

// Factory helpers for creating GameLoop instances with type deduction
namespace Runtime {
namespace Core {

template<typename UpdateFn, typename RenderFn>
auto MakeGameLoop(UpdateFn&& update, RenderFn&& render) {
    using U = std::decay_t<UpdateFn>;
    using R = std::decay_t<RenderFn>;
    return GameLoop<U, R>(std::forward<UpdateFn>(update), std::forward<RenderFn>(render));
}

template<typename UpdateFn, typename RenderFn, typename FixedUpdateFn>
auto MakeGameLoop(UpdateFn&& update, RenderFn&& render, FixedUpdateFn&& fixedUpdate) {
    using U = std::decay_t<UpdateFn>;
    using R = std::decay_t<RenderFn>;
    using F = std::decay_t<FixedUpdateFn>;
    return GameLoop<U, R, F>(std::forward<UpdateFn>(update), std::forward<RenderFn>(render), std::forward<FixedUpdateFn>(fixedUpdate));
}

} // namespace Core
} // namespace Runtime