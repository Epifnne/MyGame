# Benchmarks

## ECS archetype iteration

```powershell
cmake --build --preset release-o3 --target ECSBenchmark
.\MyGame\ECSBenchmark.exe
```

The headless benchmark creates 100,000 entities with Transform2D/Velocity2D,
then performs 200 movement passes using `View::Each`. JSON stdout contains
storage/query names, entity count, iterations, update milliseconds, and checksum.
Setup and checksum traversal are excluded from the timed region; the process
fails if the checksum differs from 60,000,000.

With `BUILD_TESTING=ON`, CTest registers `ECSBenchmark.Checksum` (60-second
timeout). Both Linux and Windows CI/release jobs execute it with the test suite;
elapsed time is informational, not a machine-dependent performance gate.
Run it locally with `ctest --test-dir Build -R "^ECSBenchmark[.]Checksum$" --output-on-failure --no-tests=error`.

Local MinGW 13.1 Release-O3 comparison (2026-10-09), three runs each:

| Implementation and query | Update time (ms) | Median (ms) |
| --- | --- | --- |
| Previous indexed optional pools + entity snapshots/Has/Get | 2115.54 / 2052.45 / 2034.48 | 2052.45 |
| Archetype typed columns + View::Each | 7.9474 / 9.3228 / 8.0493 | 8.0493 |

Both workloads use the same entity count, update arithmetic, iteration count,
compiler configuration, and final checksum. This compares the complete query
hot path (storage **and** API), not storage alone; it does not measure structural
churn, mixed archetypes, startup, memory usage, or whole-frame performance.
The local median speedup is approximately 255x for this homogeneous microbenchmark;
rerun on the target machine before drawing broader performance conclusions.

## Physics Benchmark

Headless deterministic workload for collision and solver baselines.

Build from the repository root:

```powershell
cmake --preset release-o3
cmake --build --preset release-o3 --target PhysicsBenchmark
```

Run a fixed workload:

```powershell
.\MyGame\PhysicsBenchmark.exe --scenario boxfield --bodies 500 --frames 600 --seed 1337 --output .\Build-release\physics_benchmark_500.json
```

Generate the full scenario-matrix baseline in one JSON:

```powershell
.\MyGame\PhysicsBenchmark.exe --scenario all --output .\Build-release\physics_benchmark_baseline.json
```

## Scenarios

| Scenario | Description |
| --- | --- |
| `boxfield` | Default jittered box grid dropped onto the ground. |
| `stack3` / `stack5` | Box towers of height 3 / 5 for stacking stability and solver cost. |
| `hull` | Seeded 32-vertex convex hulls; use `--bodies 100/500/1000` for the hull matrix. |
| `trigger` | Static trigger volumes with dynamic spheres passing through. |
| `churn` | Constant-population create/destroy churn (5% of bodies per frame). |
| `ccd` | High-speed spheres (-250 m/s) with CCD enabled; settled spheres are relaunched each frame so the workload stays continuous. |
| `all` | Run every scenario above and emit a combined baseline report. |

Options: `--scenario`, `--bodies`, `--frames`, `--warmup`, `--seed`, `--workers`, `--serial-narrowphase`, `--serial-islands`, `--no-sleep`, `--output`.

- `--workers N`: physics worker count for the parallel narrow-phase (Phase 6) and the parallel island solver (Phase 8); `0` (default) resolves to hardware concurrency, `1` is the fully serial deterministic fallback.
- `--serial-narrowphase`: flag (no value) forcing in-line narrow-phase execution on the calling thread.
- `--serial-islands`: flag (no value) forcing in-line per-island solver execution on the calling thread.
- `--no-sleep`: flag (no value) disabling Phase 9 island sleep (enabled by default) for sleep on/off comparisons.

The JSON report (`schemaVersion` 2) contains a `version` block (project version, compiler, build type, git commit/dirty, UTC timestamp), per-scenario config (including `physicsWorkers`/`parallelNarrowphase`/`parallelIslandSolver`/`sleepEnabled`), static/dynamic leaf counts, broad-phase candidates, narrow-phase and GJK/EPA counters, manifold/contact totals, final-state checksum, and P50/P95/P99 timings (including `islandBuild` since Phase 7). Phase 6 telemetry lands in `futureMetrics.parallelNarrowphase` (`workerCount`, `totalJobs`, `workerBusyMsTotal`, `tailWaitMsTotal`); worker busy time is a CPU-time sum and is never mixed with wall-clock percentiles. Phase 7 island metrics land in `futureMetrics.islands` (`islandCount`, `maxIslandBodyCount`, snapshot of the last sub-step). Phase 8 island solver telemetry lands in `futureMetrics.parallelIslandSolver` (same field semantics as the narrow-phase block; jobs are island batches scheduled by constraint count). Phase 9 sleep metrics land in `futureMetrics.sleep` (`awakeBodies`/`sleepingBodies`, snapshot of the last fixed step).

Note: baseline numbers recorded from a Debug build are only comparable against the same build configuration; formal performance acceptance runs in Release per the plan.

## Release-O3 stacking acceptance

```powershell
cmake --preset release-o3
cmake --build --preset release-o3 --target PhysicsBenchmark Physics_Test FieldRainBenchmark BoxStackSample
.\MyGame\BoxStackSample.exe --edge-cradle
.\MyGame\tests\Physics_Test.exe --gtest_filter=StackPipelineTest.*
.\MyGame\FieldRainBenchmark.exe default 7200
```

The cradle contains two three-high dynamic columns and a fourth-level diamond
with persistent non-face support. Key `4` selects it, and `S` disables sleep
without changing solver tuning. Key `2` selects the five-box falling tower.

`FieldRainBenchmark` preserves the sample's overlapping random spawns.
Its optional second argument is a positive frame count (default 1800);
the first argument `0` disables restitution, while `default` keeps sample materials.
It reports step P95/peak, actual floor penetration, sleeping population, TOI budget
exhaustion, and the worst pair's anchors. Failures in sink/energy/settled-floor
quality return a nonzero exit code.

PhysicsBenchmark schema version 2 adds `totals.satCalls` and
`totals.primitiveCalls` alongside GJK/EPA counters. Box pairs now use SAT and
sphere primitive pairs use analytic witnesses; zero GJK calls no longer mean
that no geometric query ran.

See the [sample overview](../Readme.md#testing--benchmarks) for the independent
sample directories and the optional local Jolt comparison executable.
