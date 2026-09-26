# Phase 8: 并行 Island Solver

完成日期：2026-09-23。对应 plan Phase 8 步骤 31-33。

## 设计

求解流水线输入从"扁平 touching 列表"切换为 Phase 7 的 Island 集合：

```
Detect -> BuildIslands(只读) -> PrepareIslandConstraints(主线程)
       -> SolveIslandConstraints(派发/串行回退) -> CommitSolvedContacts(主线程)
```

- **约束按岛分组**。`PrepareIslandConstraints` 按岛构建器的稳定岛序把
  `PreparedContactConstraint` 连续排入 `m_preparedConstraints`，每岛记录
  `[begin, end)` 区间。岛内约束保持工作列表（PairKey）升序，因此每个岛的
  Gauss-Seidel 序列与 Phase 4-7 的扁平串行顺序逐约束一致；岛间无共享可写
  body（见下），跨岛执行顺序不影响结果。
- **调度**。岛按约束数从大到小稳定排序（并列保持岛序）；约束数达到
  `PhysicsSettings::islandSolverMinConstraintsPerJob`（初值 32）的岛成为独立
  任务，小岛贪心装箱到该下限，降低尾部等待。任务经
  `JobSystem::ParallelForRange(jobCount, 1, fn)` 派发，主线程参与。
- **任务内容**。每个岛任务依次执行 WarmStart 一次、N 次
  SolveVelocityIteration、ResolvePosition 一次——即岛粒度内仍是完整串行
  Sequential Impulse，同一岛内不做约束图着色（plan 步骤 32）。
- **提交**。barrier 后主线程按稳定存储序执行 `CommitSolvedImpulses` 与固定步
  冲量汇总；事件与 `Contacts()` 发布路径不变，与任务完成顺序无关。
- **串行回退**。`workerCount == 1` 或 `SetParallelIslandSolverEnabled(false)`
  时全部任务在调用线程内联执行，调度结构不变，结果位级一致。

## 无数据竞争的结构论证

- 岛构建器保证每个动态 body 恰好属于一个岛，任务间动态 body 集合互斥。
- 共享静态体（如多个岛共同的地面）在求解期间**纯读取**：`ApplyLinearImpulse`
  /`ApplySolverAngularImpulse` 对静态体提前返回，`ApplyPositionalCorrection`
  只对非静态端调用 `SetPositionInternal`。
- worker 不写 BVH、Pair 池结构或事件列表；`Midphase::PairAt` 是纯索引访问，
  各任务只读写自己岛的 manifold slot 与 prepared 约束。
- **运行时断言（验收门）**：Debug 构建下每个岛任务先用当前 `queryEpoch` 作为
  ticket 对本岛全部动态 body 做原子戳记交换，重复戳记立即 assert。本阶段全部
  测试与基准均在 Debug 下跑过，断言从未触发。

## 遥测口径

`PhysicsStepStats` 新增（每固定步）：`islandSolverJobCount`（派发 chunk 数，
含每参与线程一次越界认领，与窄相口径一致）、`islandSolverWorkerCount`（参与
线程数快照最大值）、`islandSolverWorkerBusyMilliseconds`（worker CPU 时间
加和）、`islandSolverTailWaitMilliseconds`（各 substep 最大尾等加和）。墙钟
时间仍在 `solverMilliseconds`，二者禁止混加。Benchmark 输出
`config.parallelIslandSolver` 与 `futureMetrics.parallelIslandSolver`，新增
`--serial-islands` 开关。

## 验收结果

- `Tests/Physics/Physics_ParallelIslandSolverTest.cpp` 4 个用例通过：1/N
  worker 位级一致（位置/姿态/速度/接触点/法线/累计冲量/固定步总冲量/事件/
  求解次数/岛快照）、开关双态位级一致、共享静态地面双堆叠 1 vs 4 worker 位级
  一致、遥测分块与串行回退正确。完整 ctest 116/116 通过（1 个 Phase 4 已知
  禁用）。
- Debug boxfield 同种子三跑（1 worker / 8 worker / 8 worker 关岛并行）：
  candidates 106080、contactPoints 34235、epaFailures 960、positionChecksum
  全部位级一致，且与 Phase 7 基线（`Build/p7_box_w30.json`）逐计数一致——岛
  化求解不改变串行语义。
- TSan 在本平台（Windows+MinGW GCC）不可用，沿用 Phase 6 替代方案（结构论证
  + 确定性测试 + 运行时戳记断言）。

## Release 性能（MinGW GCC 13.1，240 帧采样 + 30 预热，dt=1/120）

| 场景 | solver P95 w1→w8 | total P95 w1→w8 | 岛结构 |
| --- | --- | --- | --- |
| stack5 --bodies 320（64 个独立 5 箱岛） | 0.410→0.226 ms（-45%） | 2.60→1.03 ms（-60%） | 64 岛，最大 5 body |
| hull --bodies 500（500 单体岛） | 0.706→0.321 ms（-55%） | 4.21→1.52 ms（-64%） | 500 岛，最大 1 body |
| boxfield（86 岛） | 0.144→0.127 ms（-12%） | 0.98→0.60 ms（-39%） | 86 岛，最大 4 body |

stack5 同配置重复运行 total P95 波动约 18%，并行收益远超波动。单个巨大岛
不承诺线性加速（plan 步骤 33），本矩阵未包含该形态。worker CPU 时间、job 数
与尾等见 `Build-release/p8_rel_*.json` 的 `futureMetrics.parallelIslandSolver`。

## 有意行为说明

- 无算法变化：每岛内的求解序列与扁平串行完全一致，跨岛顺序不影响结果，
  因此任何既有轨迹/checksum 保持不变（已用 Phase 7 基线验证）。
- 触发器与 static-static 接触的跳过从 Prepare 阶段移到岛构建器（这两类接触
  本就不成边），效果等价；位置修正同理只作用于岛内约束（原实现内部 guard）。
- `velocitySolverPassCount`/`positionSolverPassCount` 计数口径不变。
