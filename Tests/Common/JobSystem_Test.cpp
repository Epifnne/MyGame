#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <vector>

#include "Core/JobSystem.h"

namespace {

using Runtime::Core::JobSystem;

class JobSystemTest : public ::testing::Test {
protected:
    void TearDown() override {
        JobSystem::Get().Shutdown();
    }
};

// Acceptance: every index in [0, count) is processed exactly once.
TEST_F(JobSystemTest, ParallelForRangeExecutesEveryIndexExactlyOnce) {
    constexpr std::size_t kCount = 100000;
    auto& jobs = JobSystem::Get();
    jobs.Initialize(4);

    std::vector<std::atomic<uint32_t>> hits(kCount);
    jobs.ParallelForRange(kCount, 64, [&hits](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            hits[i].fetch_add(1, std::memory_order_relaxed);
        }
    });

    for (std::size_t i = 0; i < kCount; ++i) {
        EXPECT_EQ(hits[i].load(std::memory_order_relaxed), 1u) << "index " << i;
    }
}

// Acceptance: worker counts 1 / 2 / 4 / hardware concurrency produce identical
// results (also exercises repeated Initialize with different pool sizes).
TEST_F(JobSystemTest, ResultsAreIdenticalAcrossWorkerCounts) {
    constexpr std::size_t kCount = 4096;
    const uint32_t hardware = std::max(1u, std::thread::hardware_concurrency());
    const uint32_t counts[] = {1u, 2u, 4u, hardware};

    std::vector<uint64_t> reference;
    for (const uint32_t workerCount : counts) {
        auto& jobs = JobSystem::Get();
        jobs.Initialize(workerCount);
        EXPECT_EQ(jobs.WorkerCount(), workerCount);

        std::vector<uint64_t> values(kCount, 0);
        jobs.ParallelForRange(kCount, 16, [&values](std::size_t begin, std::size_t end) {
            for (std::size_t i = begin; i < end; ++i) {
                values[i] = static_cast<uint64_t>(i) * 2654435761ull % 1000003ull;
            }
        });

        if (reference.empty()) {
            reference = values;
        } else {
            EXPECT_EQ(values, reference) << "workerCount " << workerCount;
        }
        jobs.Shutdown();
    }
}

// Acceptance: workerCount == 1 is a pure serial fallback on the calling thread.
TEST_F(JobSystemTest, SingleWorkerRunsInlineOnCallingThread) {
    auto& jobs = JobSystem::Get();
    jobs.Initialize(1);

    const std::thread::id caller = std::this_thread::get_id();
    std::thread::id observed;
    jobs.ParallelForRange(1000, 8, [&observed](std::size_t begin, std::size_t end) {
        observed = std::this_thread::get_id();
        EXPECT_EQ(begin, 0u);
        EXPECT_EQ(end, 1000u);
    });
    EXPECT_EQ(observed, caller);
}

// Acceptance: small ranges degrade to direct execution on the calling thread
// even when the pool has real workers.
TEST_F(JobSystemTest, SmallRangesRunInlineOnCallingThread) {
    auto& jobs = JobSystem::Get();
    jobs.Initialize(4);

    const std::thread::id caller = std::this_thread::get_id();
    std::thread::id observed;
    std::size_t calls = 0;
    // count <= minItemsPerJob: a single chunk, executed inline.
    jobs.ParallelForRange(8, 64, [&observed, &calls](std::size_t, std::size_t) {
        observed = std::this_thread::get_id();
        ++calls;
    });
    EXPECT_EQ(calls, 1u);
    EXPECT_EQ(observed, caller);
}

// Acceptance: empty ranges are a no-op and never deadlock.
TEST_F(JobSystemTest, EmptyRangeIsANoOp) {
    auto& jobs = JobSystem::Get();
    jobs.Initialize(4);

    std::size_t calls = 0;
    jobs.ParallelForRange(0, 16, [&calls](std::size_t, std::size_t) { ++calls; });
    EXPECT_EQ(calls, 0u);
}

// Acceptance: repeated Initialize/Shutdown cycles never deadlock, and Shutdown
// is safe to call repeatedly or without prior Initialize.
TEST_F(JobSystemTest, RepeatedInitializeAndShutdownAreSafe) {
    auto& jobs = JobSystem::Get();
    for (int round = 0; round < 3; ++round) {
        jobs.Initialize(4);
        std::atomic<std::size_t> total{0};
        jobs.ParallelForRange(512, 16, [&total](std::size_t begin, std::size_t end) {
            total.fetch_add(end - begin, std::memory_order_relaxed);
        });
        EXPECT_EQ(total.load(), 512u);
        jobs.Shutdown();
        jobs.Shutdown();
    }
    EXPECT_FALSE(jobs.IsInitialized());
}

// A throwing chunk must not deadlock the barrier; the first exception is
// rethrown on the calling thread and the pool stays usable afterwards.
TEST_F(JobSystemTest, ChunkExceptionRethrowsAndPoolSurvives) {
    auto& jobs = JobSystem::Get();
    jobs.Initialize(4);

    std::atomic<std::size_t> processed{0};
    EXPECT_THROW(
        jobs.ParallelForRange(256, 8, [&processed](std::size_t begin, std::size_t end) {
            processed.fetch_add(end - begin, std::memory_order_relaxed);
            if (begin == 0) {
                throw std::runtime_error("chunk failure");
            }
        }),
        std::runtime_error);
    // Every chunk still executed exactly once despite the failure.
    EXPECT_EQ(processed.load(), 256u);

    std::atomic<std::size_t> after{0};
    jobs.ParallelForRange(128, 8, [&after](std::size_t begin, std::size_t end) {
        after.fetch_add(end - begin, std::memory_order_relaxed);
    });
    EXPECT_EQ(after.load(), 128u);
}

// Chunk sizes respect the caller's minimum items per job.
TEST_F(JobSystemTest, ChunksRespectMinItemsPerJob) {
    auto& jobs = JobSystem::Get();
    jobs.Initialize(4);

    constexpr std::size_t kCount = 1000;
    constexpr std::size_t kMinChunk = 128;
    std::atomic<std::size_t> maxChunks{0};
    std::atomic<std::size_t> covered{0};
    jobs.ParallelForRange(kCount, kMinChunk, [&maxChunks, &covered](std::size_t begin, std::size_t end) {
        covered.fetch_add(end - begin, std::memory_order_relaxed);
        maxChunks.fetch_add(1, std::memory_order_relaxed);
    });
    EXPECT_EQ(covered.load(), kCount);
    EXPECT_LE(maxChunks.load(), (kCount + kMinChunk - 1) / kMinChunk);
}

// Shutdown with work never queued and destruction without explicit Shutdown
// must both be deadlock free (destructor covered by TearDown of other tests).
TEST_F(JobSystemTest, ShutdownWithoutWorkIsSafe) {
    auto& jobs = JobSystem::Get();
    jobs.Initialize(4);
    jobs.Shutdown();
    EXPECT_EQ(jobs.WorkerCount(), 0u);
}

} // namespace
