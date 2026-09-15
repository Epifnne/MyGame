# Phase 6：并行 Narrowphase 实施与验收记录

> 日期：2026-09-12。状态：已落地并通过全部验收门。
> 关联：[plan.md](plan.md) Phase 6 第 25–27 条、[defunction-callback-optimization.md](defunction-callback-optimization.md)（Phase 5，A/B 方法复用）。

## 实施内容

### 执行结构（plan 第 25 条）

[CollisionDetector.cpp](../Runtime/src/Physics/CollisionDetector.cpp) 的 Stage 3 拆为 `ExecuteNarrowphase`：

- **分块**：基于 Midphase 连续 work list，经 Phase 5 的 `JobSystem::ParallelForRange` 派发，初始 `minPairsPerJob = 32`（`PhysicsSettings::narrowphaseMinPairsPerJob`），主线程参与，任务数由块大小与负载自动调整（不硬限为 worker 数，允许凸包等成本不同的查询负载均衡）。
- **worker 只写自己的输出槽**：每个 work item 一个 `WorkOutput`（状态/触发标记/每对 GJK-EPA 私有统计/fresh manifold），预分配、跨帧复用容量；chunk 记录槽由原子计数认领，无任何共享 `push_back`、无锁、不改 BVH/map/body、不分发事件。worker 只读冻结的 Collider/RigidBody/Shape 快照（不读旧流形——匹配与缓存转移全部在主线程提交阶段完成）。
- **统计任务私有、barrier 后归并**：`NarrowPhase::GenerateContact` 的统计从共享 `mutable` 成员改为按调用输出参数（`NarrowPhaseQueryStats& outStats`，调用开始重置），这是同实例并发的直接前提；CCD 的栈上局部实例同步适配。
- **主线程提交**：barrier 后按 work list（稳定 PairKey 序）逐一 CommitContact/CommitSeparation/CommitQueryFailure，事件与 touching 集合顺序与串行完全一致。

### 开关面（plan 第 26 条）

- `PhysicsWorld::SetParallelNarrowphaseEnabled(bool)` / `SetPhysicsWorkerCount(uint32_t)`（workerCount 含调用线程，0=硬件并发，1=纯串行确定性回退）；默认参数收敛到 [PhysicsSettings.h](../Runtime/include/Physics/PhysicsSettings.h)（plan 相关文件清单中此前缺失的文件，本阶段补齐）。
- 串行回退（开关关闭或 workerCount=1）在调用线程内联执行同一 range 体，不经 job 队列。

### 遥测（plan 第 27 条）

`NarrowPhaseParallelStats`（`CollisionDetector::LastParallelStats()`）记录每线程 job 数、每线程 busy 耗时与尾部等待（线程完成最后一个 chunk 到 barrier 返回的最长等待）；`PhysicsStepStats` 汇聚 `narrowPhaseJobCount/narrowPhaseWorkerCount/narrowPhaseWorkerBusyMilliseconds/narrowPhaseTailWaitMilliseconds`；Benchmark JSON 在 `futureMetrics.parallelNarrowphase` 输出（worker busy 是 CPU 时间总和，与墙钟分位数分列，不混加）。Benchmark 新增 `--workers N` 与 `--serial-narrowphase` 选项（此前依赖 Phase 6 开关面的缺口，本阶段补齐）。

## 有意行为修正：GJK/EPA 失败检测的跨调用污染

旧实现用持久实例的 `LastQueryStats()` 做差值比较（`failuresAfter != failuresBefore`），失败计数携带上一次调用的状态：

1. 上次失败、本次正常 → 被误判为 QueryFailure，本次真实接触/分离被丢弃（touching 状态不更新、本轮不参与求解）；
2. 上次失败、本次也失败 → 差值为零被漏判，算法失败冒充正常分离（发 Exit、清缓存）——违反冻结契约"算法失败不冒充正常分离"。

新实现基于当次调用的独立统计（`failures != 0`），逐次判定，符合契约。**轨迹影响**：boxfield（旧 EPA 失败 877 次）接触点 34883→34235、epaFailures 877→960（失败不再被相邻失败掩盖，计数恢复真实值）；hull500 全程零失败，轨迹位级不变。按 plan Verification 第 1 条，此为有意修正，比较不变量与质量门槛而非强求旧轨迹一致；完整 ctest 103/103 通过（1 个 Phase 4 已知禁用）。

## 验收门结果

1. **1 worker 与 N worker 一致**（plan 第 27 条）：`Physics_ParallelNarrowphaseTest` 4 个用例通过——1/4 worker 的 Pair、接触点、法线、事件流、统计计数与最终状态**位级一致**；开关双态位级一致；任务私有统计归并等于串行总计；遥测分块与串行回退行为正确。Benchmark 双跑（含自动 worker）checksum 位级一致。
2. **ThreadSanitizer**：本平台（Windows + MinGW GCC 13.1）无 TSan 支持，按"可用平台"条款记录为不可用；数据竞争防护由结构保证（冻结输入、私有输出槽、原子认领、post-barrier 归并）并由确定性测试交叉验证。
3. **每线程 job 数/耗时/尾部等待**：已实现并随基准输出（见下表）。
4. **500 凸包相对单线程有可测收益**：见下表。

## 性能验收（Release，MinGW 13.1 -O3，16 线程，5 次采样取 P95 中位数）

确定性门：1 worker 与 16 worker 的 broadPhaseCandidates / contactPoints / positionChecksum 全部位级一致。

| 场景 | 阶段 | 1 worker (ms) | 16 worker (ms) | 提升 |
|---|---|---|---|---|
| hull500 | narrowPhase | 2.219 | 0.529 | **-76%** |
| hull500 | total | 3.473 | 1.811 | **-48%** |
| boxfield | narrowPhase | 0.578 | 0.359 | **-38%** |
| boxfield | total | 0.855 | 0.788 | -8% |

并行遥测（240 采样帧累计，中位数）：

| 场景 | totalJobs | workerBusyMs | tailWaitMs |
|---|---|---|---|
| hull500 (16 worker) | 4010 | 650.0 | 31.98 |
| boxfield (16 worker) | 3360 | 198.6 | 37.99 |

采样期间发现并行基准偶发挂起，根因为 chunk 遥测记录槽按实际 chunk 数分配：JobSystem 每个参与线程耗尽任务前会多认领一个越界 chunk（`nextChunk.fetch_add` 超额），越界写导致 worker 崩溃卡死 barrier。已按 实际chunk数+workerCount 预留修复，修复后全部采样通过。

## 结论与后续

- Phase 6 完成，Agent 9 任务收口；Agent 10（Physics Island）可启动，Agent 11（并行 Island Solver）依赖 Agent 9/10。
- worker CPU 时间总和高于串行墙钟属预期（并行调度与等待开销），正式 Phase 11 验收按 plan 第 42 条墙钟与 CPU 时间分列。
