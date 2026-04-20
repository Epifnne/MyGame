// Engine/include/Core/Time.h
#pragma once
#include <chrono>

namespace Runtime {
namespace Core {

class Time {
public:
    static Time& Get() {
        static Time instance;
        return instance;
    }

    void Update() {
        auto currentTime = std::chrono::high_resolution_clock::now();
        m_deltaTime = std::chrono::duration<float>(currentTime - m_lastFrameTime).count();
        
        // limit max deltaTime
        if (m_deltaTime > m_maxDeltaTime) {
            m_deltaTime = m_maxDeltaTime;
        }
        
        m_lastFrameTime = currentTime;
        m_totalTime += m_deltaTime;
        
        // FPS calculation
        m_frameCount++;
        m_fpsAccumulator += m_deltaTime;
        if (m_fpsAccumulator >= 1.0f) {
            m_currentFPS = m_frameCount;
            m_frameCount = 0;
            m_fpsAccumulator -= 1.0f;
        }
    }
    
    float GetDeltaTime() const { return m_deltaTime; }
    float GetTotalTime() const { return m_totalTime; }
    int GetFPS() const { return m_currentFPS; }
    
private:
    Time() = default;
    
    using Clock = std::chrono::high_resolution_clock;
    Clock::time_point m_lastFrameTime = Clock::now();
    
    float m_deltaTime = 0.0f;
    float m_totalTime = 0.0f;
    float m_maxDeltaTime = 0.033f;  // 30 FPS limit
    
    // FPS calculation
    int m_frameCount = 0;
    float m_fpsAccumulator = 0.0f;
    int m_currentFPS = 0;
};


}// namespace Core
}// namespace Runtime