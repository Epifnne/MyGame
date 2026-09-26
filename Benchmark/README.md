# Physics Benchmark

Headless deterministic workload for collision and solver baselines.

Build from the repository root:

```powershell
& 'D:\Qt\Tools\CMake_64\bin\cmake.exe' --preset mingw-debug
& 'D:\Qt\Tools\CMake_64\bin\cmake.exe' --build --preset mingw-debug --target PhysicsBenchmark
```

Run a fixed workload:

```powershell
.\MyGame\PhysicsBenchmark.exe --scenario boxfield --bodies 500 --frames 600 --seed 1337 --output .\Build\physics_benchmark_500.json
```

Generate the full scenario-matrix baseline in one JSON:

```powershell
.\MyGame\PhysicsBenchmark.exe --scenario all --output .\Build\physics_benchmark_baseline.json
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

Options: `--scenario`, `--bodies`, `--frames`, `--warmup`, `--seed`, `--workers`, `--serial-narrowphase`, `--serial-islands`, `--output`.

- `--workers N`: physics worker count for the parallel narrow-phase (Phase 6) and the parallel island solver (Phase 8); `0` (default) resolves to hardware concurrency, `1` is the fully serial deterministic fallback.
- `--serial-narrowphase`: flag (no value) forcing in-line narrow-phase execution on the calling thread.
- `--serial-islands`: flag (no value) forcing in-line per-island solver execution on the calling thread.

The JSON report (`schemaVersion` 2) contains a `version` block (project version, compiler, build type, git commit/dirty, UTC timestamp), per-scenario config (including `physicsWorkers`/`parallelNarrowphase`/`parallelIslandSolver`), static/dynamic leaf counts, broad-phase candidates, narrow-phase and GJK/EPA counters, manifold/contact totals, final-state checksum, and P50/P95/P99 timings (including `islandBuild` since Phase 7). Phase 6 telemetry lands in `futureMetrics.parallelNarrowphase` (`workerCount`, `totalJobs`, `workerBusyMsTotal`, `tailWaitMsTotal`); worker busy time is a CPU-time sum and is never mixed with wall-clock percentiles. Phase 7 island metrics land in `futureMetrics.islands` (`islandCount`, `maxIslandBodyCount`, snapshot of the last sub-step). Phase 8 island solver telemetry lands in `futureMetrics.parallelIslandSolver` (same field semantics as the narrow-phase block; jobs are island batches scheduled by constraint count). Sleep metrics remain marked unavailable until that stage is implemented.

Note: baseline numbers recorded from a Debug build are only comparable against the same build configuration; formal performance acceptance runs in Release per the plan.
