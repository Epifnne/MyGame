#include <gtest/gtest.h>
#include "Core/GameLoop.h"
#include "Core/Time.h"
#include <limits>
#include <vector>

using Runtime::Core::Time;
using Runtime::Core::MakeGameLoop;
using namespace std::chrono_literals;

TEST(TimeTest, ClampsSimulationDeltaButCountsFPSWithWallTime) {
    Time time;
    const Time::Clock::time_point start{};
    time.Reset(start);
    for (int i = 1; i <= 4; ++i) time.Update(start + i * 250ms);
    EXPECT_FLOAT_EQ(time.GetDeltaTime(), 0.033f);
    EXPECT_NEAR(time.GetTotalTime(), 4.0 * 0.033f, 1e-8);
    EXPECT_DOUBLE_EQ(time.GetElapsedTime(), 1.0);
    EXPECT_EQ(time.GetFPS(), 4);
    EXPECT_EQ(time.GetFrameCount(), 4u);
}

TEST(TimeTest, RejectsInvalidConfigurationAndBackwardsSamples) {
    Time time;
    EXPECT_THROW(time.SetFixedTimeStep(0.0f), std::invalid_argument);
    EXPECT_THROW(time.SetFixedTimeStep(-1.0f), std::invalid_argument);
    EXPECT_THROW(time.SetFixedTimeStep(std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
    EXPECT_THROW(time.SetMaxDeltaTime(0.0f), std::invalid_argument);
    EXPECT_THROW(time.SetMaxDeltaTime(std::numeric_limits<float>::infinity()), std::invalid_argument);
    const Time::Clock::time_point start{};
    time.Reset(start);
    EXPECT_THROW(time.Update(start - 1ms), std::invalid_argument);
    EXPECT_EQ(time.GetFrameCount(), 0u);
}

TEST(TimeTest, FixedAccumulatorLivesInTimeAndKeepsRemainder) {
    Time time;
    const Time::Clock::time_point start{};
    time.SetFixedTimeStep(0.01f);
    time.Reset(start);
    time.Update(start + 25ms);
    EXPECT_TRUE(time.ConsumeFixedStep());
    EXPECT_TRUE(time.ConsumeFixedStep());
    EXPECT_FALSE(time.ConsumeFixedStep());
    EXPECT_NEAR(time.GetAccumulatedTime(), 0.005, 1e-8);
    time.Reset(start);
    EXPECT_DOUBLE_EQ(time.GetAccumulatedTime(), 0.0);
    EXPECT_EQ(time.GetFPS(), 0);
    EXPECT_EQ(time.GetFrameCount(), 0u);
}

TEST(TimeTest, GameLoopCallbacksUseSameTimeSampleAndPreserveOrder) {
    Time time;
    std::vector<int> order;
    auto loop = MakeGameLoop(
        [&](float dt) { EXPECT_FLOAT_EQ(dt, time.GetDeltaTime()); order.push_back(1); },
        [&] { order.push_back(3); },
        [&](float dt) { EXPECT_FLOAT_EQ(dt, time.GetFixedDeltaTime()); order.push_back(2); },
        time);
    const Time::Clock::time_point start{};
    loop.SetFixedTimeStep(0.01f);
    loop.Start(start);
    loop.Tick(start + 25ms);
    EXPECT_EQ(order, (std::vector<int>{1, 2, 2, 3}));
    EXPECT_EQ(loop.GetFrameCount(), 1u);
}

TEST(TimeTest, EngineStyleDispatchDoesNotSampleTimeTwice) {
    auto& time = Time::Get();
    int updates = 0;
    auto loop = MakeGameLoop(
        [&](float dt) { EXPECT_FLOAT_EQ(dt, time.GetDeltaTime()); ++updates; },
        [] {});
    const Time::Clock::time_point start{};
    loop.Start(start);
    time.Update(start + 16ms);
    loop.TickCurrentFrame();
    EXPECT_EQ(updates, 1);
    EXPECT_EQ(time.GetFrameCount(), 1u);
    EXPECT_FLOAT_EQ(time.GetDeltaTime(), 0.016f);
}

TEST(TimeTest, PauseAndResumeExcludePauseDurationAndRestartResetsStatistics) {
    Time time;
    int updates = 0;
    auto loop = MakeGameLoop([&](float) { ++updates; }, [] {}, nullptr, time);
    const Time::Clock::time_point start{};
    loop.Start(start);
    loop.Tick(start + 10ms);
    loop.Pause();
    loop.Tick(start + 1s);
    EXPECT_EQ(updates, 1);
    loop.Resume(start + 2s);
    loop.Tick(start + 2s + 10ms);
    EXPECT_FLOAT_EQ(time.GetDeltaTime(), 0.01f);
    EXPECT_NEAR(time.GetTotalTime(), 0.02, 1e-8);
    EXPECT_NEAR(time.GetElapsedTime(), 0.02, 1e-8);
    EXPECT_EQ(updates, 2);
    loop.Stop();
    loop.Tick(start + 3s);
    EXPECT_EQ(updates, 2);
    loop.Start(start + 4s);
    EXPECT_EQ(loop.GetFrameCount(), 0u);
    EXPECT_DOUBLE_EQ(time.GetTotalTime(), 0.0);
}
