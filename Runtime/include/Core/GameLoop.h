// Engine/include/Core/GameLoop.h
#pragma once

#include "Core/Time.h"
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace Runtime {
namespace Core {

enum class GameLoopState : uint8_t {
    Running,
    Paused,
    Stopped
};

template<typename UpdateFn, typename RenderFn, typename FixedUpdateFn = std::nullptr_t>
class GameLoop {
public:
    GameLoop(UpdateFn update, RenderFn render, FixedUpdateFn fixedUpdate = nullptr,
             Time& time = Time::Get())
        : m_update(std::move(update))
        , m_render(std::move(render))
        , m_fixedUpdate(std::move(fixedUpdate))
        , m_time(time)
    {}
    
    void Start(Time::Clock::time_point now = Time::Clock::now()) {
        m_state = GameLoopState::Running;
        m_time.Reset(now);
    }
    
    void Stop() { m_state = GameLoopState::Stopped; }
    void Pause() { if (m_state == GameLoopState::Running) m_state = GameLoopState::Paused; }
    void Resume(Time::Clock::time_point now = Time::Clock::now()) {
        if (m_state == GameLoopState::Paused) {
            m_time.Rebase(now);
            m_state = GameLoopState::Running;
        }
    }
    
    GameLoopState GetState() const { return m_state; }
    
    void Tick(Time::Clock::time_point now = Time::Clock::now()) {
        if (m_state != GameLoopState::Running) return;
        m_time.Update(now);
        TickCurrentFrame();
    }

    // Engine samples Time once before frame work; standalone users call Tick().
    void TickCurrentFrame() {
        if (m_state != GameLoopState::Running) return;
        m_update(m_time.GetDeltaTime());
        if constexpr (!std::is_same_v<FixedUpdateFn, std::nullptr_t>) {
            while (m_time.ConsumeFixedStep()) {
                m_fixedUpdate(m_time.GetFixedDeltaTime());
            }
        }
        m_render();
    }
    
    void SetFixedTimeStep(float step) { m_time.SetFixedTimeStep(step); }
    void SetMaxDeltaTime(float maxDt) { m_time.SetMaxDeltaTime(maxDt); }
    uint64_t GetFrameCount() const { return m_time.GetFrameCount(); }
    
private:
    UpdateFn m_update;
    RenderFn m_render;
    FixedUpdateFn m_fixedUpdate;
    
    Time& m_time;
    GameLoopState m_state = GameLoopState::Stopped;
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
auto MakeGameLoop(UpdateFn&& update, RenderFn&& render, FixedUpdateFn&& fixedUpdate,
                  Time& time = Time::Get()) {
    using U = std::decay_t<UpdateFn>;
    using R = std::decay_t<RenderFn>;
    using F = std::decay_t<FixedUpdateFn>;
    return GameLoop<U, R, F>(std::forward<UpdateFn>(update), std::forward<RenderFn>(render),
                           std::forward<FixedUpdateFn>(fixedUpdate), time);
}

} // namespace Core
} // namespace Runtime