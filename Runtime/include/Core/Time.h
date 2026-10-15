#pragma once

#include <chrono>
#include <cstdint>

namespace Runtime::Core {

class Time {
public:
    using Clock = std::chrono::steady_clock;

    static Time& Get() {
        static Time instance;
        return instance;
    }

    Time() { Reset(); }
    void Reset(Clock::time_point now = Clock::now());
    void Rebase(Clock::time_point now = Clock::now());
    void Update(Clock::time_point now = Clock::now());
    bool ConsumeFixedStep();
    void SetFixedTimeStep(float step);
    void SetMaxDeltaTime(float maxDt);

    float GetDeltaTime() const { return m_deltaTime; }
    double GetTotalTime() const { return m_totalTime; }
    double GetElapsedTime() const { return m_elapsedTime; }
    double GetAccumulatedTime() const { return m_accumulator; }
    float GetFixedDeltaTime() const { return m_fixedTimeStep; }
    int GetFPS() const { return m_currentFPS; }
    uint64_t GetFrameCount() const { return m_frameCount; }

private:
    Clock::time_point m_lastFrameTime;
    float m_deltaTime = 0.0f;
    float m_fixedTimeStep = 1.0f / 30.0f;
    float m_maxDeltaTime = 0.033f;
    double m_totalTime = 0.0;
    double m_elapsedTime = 0.0;
    double m_accumulator = 0.0;
    double m_fpsAccumulator = 0.0;
    uint64_t m_frameCount = 0;
    uint64_t m_fpsFrameCount = 0;
    int m_currentFPS = 0;
};

} // namespace Runtime::Core
