#include "Core/Time.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace Runtime::Core {

void Time::Reset(Clock::time_point now) {
    m_lastFrameTime = now;
    m_deltaTime = 0.0f;
    m_totalTime = 0.0;
    m_elapsedTime = 0.0;
    m_accumulator = 0.0;
    m_fpsAccumulator = 0.0;
    m_frameCount = 0;
    m_fpsFrameCount = 0;
    m_currentFPS = 0;
}

void Time::Rebase(Clock::time_point now) {
    m_lastFrameTime = now;
    m_deltaTime = 0.0f;
}

void Time::Update(Clock::time_point now) {
    if (now < m_lastFrameTime) throw std::invalid_argument("Time cannot move backwards");
    const double elapsed = std::chrono::duration<double>(now - m_lastFrameTime).count();
    m_lastFrameTime = now;
    m_deltaTime = static_cast<float>(std::min(elapsed, static_cast<double>(m_maxDeltaTime)));
    m_totalTime += m_deltaTime;
    m_elapsedTime += elapsed;
    m_accumulator += m_deltaTime;
    ++m_frameCount;
    ++m_fpsFrameCount;
    // FPS uses wall-clock time, not the clamped simulation delta.
    m_fpsAccumulator += elapsed;
    if (m_fpsAccumulator >= 1.0) {
        const auto rate = std::round(static_cast<double>(m_fpsFrameCount) / m_fpsAccumulator);
        m_currentFPS = static_cast<int>(std::min(rate, static_cast<double>(std::numeric_limits<int>::max())));
        m_fpsFrameCount = 0;
        m_fpsAccumulator = 0.0;
    }
}

bool Time::ConsumeFixedStep() {
    if (m_accumulator < m_fixedTimeStep) return false;
    m_accumulator -= m_fixedTimeStep;
    return true;
}

void Time::SetFixedTimeStep(float step) {
    if (!std::isfinite(step) || step <= 0.0f) {
        throw std::invalid_argument("Fixed time step must be finite and positive");
    }
    m_fixedTimeStep = step;
}

void Time::SetMaxDeltaTime(float maxDt) {
    if (!std::isfinite(maxDt) || maxDt <= 0.0f) {
        throw std::invalid_argument("Maximum delta time must be finite and positive");
    }
    m_maxDeltaTime = maxDt;
}

} // namespace Runtime::Core
