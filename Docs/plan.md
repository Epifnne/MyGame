## Plan: 物理碰撞优化总路线

在保留 GJK/EPA 与顺序冲量求解框架的基础上，依次完成基线统计、每 substep 单次检测、双 BVH、完整 Midphase、持久流形与累计冲量、预计算角响应、固定线程池、并行 Narrowphase、Physics Island、并行 Island Solver、睡眠/唤醒和 CCD 整合。刚体继续以角动量 L 作为本地物理权威状态，但求解热路径通过预计算响应直接累加 delta v/delta omega，并同步 L；未来网络快照使用 orientation + angularVelocity。

**当前状态（2026-09-08）**

- 已完成：Phase 0 步骤 2，即每个普通 substep 单次碰撞检测、N 次速度求解、一次位置修正，以及 legacy A/B 开关和调用次数测试。
- 已完成（2026-09-08）：Phase 2 完整 Midphase 与持久流形。`Midphase.h/.cpp` 落地 PairKey→PairHandle（代数）映射 + 连续 slot/free-list 池，维护 New/Persisting/Removed、`lastSeenQueryEpoch`、Collider 身份/revision、材质值快照、body 外部姿态/结构修订号、`hadContact`、`needsNarrowphase`、持久 `ContactManifold` 和连续工作列表；`ContactPoint` 扩展 localPointA/B、accumulatedNormalImpulse、accumulatedTangentImpulse(vec2)、旧切线基、cachedDt 与 TOI 标记，锚点距离+法线阈值确定性一对一匹配（并列取最小旧索引）；`CollisionDetector` 拆分为 Broadphase→Midphase→Narrowphase→主线程提交四段，提交时匹配旧点并转移缓存；GJK/EPA 失败不冒充分离（不改 touching 状态、不发 Exit、本轮不参与求解）；`PhysicsWorld` 接线 fixedStepId/queryEpoch、Enter/Stay/Exit 事件（Stay 每固定步至多一次，步内 Enter 后 Exit 不被吞）、PairKey 固定步接触汇总（补丁 3，`MergeStepContacts` 已删除）、`SolveContactsIterative` 更名 `DetectAndSolveContacts`（补丁 4）、`normalImpulse` 升级为最近有效接触子步累计口径且 Trigger 冲量恒零（补丁 8 的 Phase 2 部分）；位置修正改走 `SetPositionInternal`，外部 `SetPosition/SetOrientation` 按 teleport 清缓存；Benchmark 接入真实 midphase 指标（补丁 7）。验收测试 `Physics_MidphaseTest.cpp` 19 个用例全部通过，完整 ctest 73/73 通过，boxfield 同种子双跑 checksum 位级一致。
- 已完成（2026-09-08）：Phase 3 刚体角状态与预计算响应。`RigidBody` 缓存世界逆惯量 `m_inverseInertiaTensorWorld`，由构造、`SetOrientation`、`SetInertiaTensorDiagonal`、`SetStatic` 和 `Integrate` 姿态更新统一刷新，静态体严格为零，`SetMass` 不触发刷新；`ApplyAngularImpulse` 改为增量式 `L += impulse`、`omega += cache * impulse`；新增 solver 专用入口 `ApplySolverAngularImpulse(angularImpulse, deltaOmega)`，一次调用同步累加 L 与预计算 delta omega；`ContactSolver::ApplyImpulsePair` 改用与有效质量相同的逆惯量矩阵预计算 delta omega 并经该入口写入，求解器不再经重组装路径写 omega；`SetAngularVelocity` 改为先推导 L 再经缓存回推 omega（静态体得到严格零）；网络快照契约（orientation + angularVelocity，先 `SetOrientation` 后 `SetAngularVelocity`，惯量由资源版本保证一致）已在 `RigidBody.h` 注释冻结，未修改网络消息。验收测试 `Physics_RigidBodyTest.cpp` 6 个用例通过，完整 ctest 79/79 通过，boxfield 同种子双跑 checksum 位级一致；与 Phase 2 基线相比仅 float 级轨迹漂移（broadPhaseCandidates/narrowPhaseTests 不变，contactPoints 27947→27830，manifolds 25549→25474）。
- 基本完成：Phase 0 步骤 1 的统计基础设施和无窗口固定种子基准程序；尚未形成完整的 3/5 箱、100/500/1000 凸包、Trigger、创建销毁、CCD 场景矩阵及正式基线文件。
- 部分完成：累计冲量语义。`normalImpulse` 现在是当前 substep 内实际施加的累计冲量（零 warm start 初值），持久缓存经锚点匹配跨帧转移；真正的累计增量形式求解（delta lambda 夹紧、摩擦圆盘、Warm Start）仍属 Phase 4。
- 已完成（2026-09-08）：Phase 4 约束准备、Warm Start 与累计冲量。`ContactSolver` 重构为 Prepare→WarmStart→SolveVelocityIteration N→ResolvePosition→CommitSolvedImpulses 流水线；`PreparedContactConstraint/PreparedContactPoint` 为 substep 临时结构，与持久 `ContactPoint` 分离；有效质量与施加响应同源（`BuildAxis` 预计算 `Iworld^-1 * (r cross axis)` 与有效质量分母共享逆惯量矩阵）；WarmStart 按 `dtNew/cachedDt` 缩放、切线基重投影、摩擦圆盘夹紧，TOI 冲击（当前或缓存来源）初值为零；速度求解改为累计增量形式（法向 `lambda_n >= 0`，切向 `||lambda_t|| <= mu * lambda_n`，负 delta 撤回过大初值）；Restitution 为 Prepare 阶段一次性速度 bias（低速阈值 1.0 m/s 抑制微反弹），不再逐迭代重复应用；one-sided 法线在 Prepare 重定向，正反面判定由 closing velocity 沿重定向法线表达；旧 `bounce boost` 与非物理 `angularScale` 已移除（有意行为修正）；`PhysicsWorld` 删除 `SetLegacyIterativeDetectionEnabled` 迁移路径（Phase 0 的 A/B 工具收尾）；`Midphase` 的缓存解释数据（旧切线基/cachedDt/TOI 来源）随匹配转移，`CommitContact` 不再重新盖印；`ContactManifold` 新增 `fixedStepNormalImpulse`/`fixedStepTangentImpulse` 固定步总冲量汇总（补丁 8 完成），Trigger 恒零；GameApp 遥测按字段口径展示。验收测试 `Physics_ContactSolverTest.cpp` 13 个用例通过（含 1 个 DISABLED 堆叠质量测试，见下）。完整 ctest 90/90 通过（1 禁用），boxfield 同种子双跑 checksum 位级一致（34883 接触点）。
- 已知限制（Phase 4 验收门第 21 条）：低迭代堆叠场景（`LowIterationStackStaysWithinQualityBounds`）在 4 次迭代下最终坍塌。根因是多点流形 + warm start + 顺序冲量的架构限制：零角臂质心点（Phase 2/3 保留的代表性接触点）在 Gauss-Seidel 中作为负载汇点维持短期稳定，但微小倾斜累积后求解器无法在点间重新分配负载，倾斜被放大直至坍塌。旧实现（Phase 3）通过允许 0.171 穿透和未收敛速度维持表观稳定；新实现穿透 0.011、速度收敛，但位置漂移。块求解器或稳定 feature ID（plan 后续增强）是根本修复方向。详见 `Docs/phase4-investigation.md`。
- 已完成（2026-09-11）：Phase 5 固定 JobSystem。`Runtime/include/Core/JobSystem.h` 与 `Runtime/src/Core/JobSystem.cpp` 落地：固定 `std::jthread` worker 池（workerCount 含调用线程，1 为纯串行回退）、条件变量批量派发（每 chunk 一张 ticket，共享同一份 Job 快照，杜绝跨代别名）、帧级 barrier（`ParallelForRange` 返回即完成）、主线程参与、空任务/重复初始化/安全关闭；chunk 异常被捕获计数、barrier 不挂起并在调用线程重抛首个异常；小任务（单 chunk 或串行配置）退化为调用线程直接执行；无任何每帧 `std::async`。验收测试 `Tests/Common/JobSystem_Test.cpp` 9 个用例通过（每索引恰执行一次、1/2/4/硬件并发结果一致、串行/小任务调用线程内联、关闭无死锁、minItemsPerJob 生效、异常后池仍可用），完整 ctest 99/99 通过（1 个 Phase 4 已知禁用）。
- 已完成（2026-09-12）：Phase 6 并行 Narrowphase。`CollisionDetector` Stage 3 拆分为 `ExecuteNarrowphase`：Midphase 连续 work list 按 `PhysicsSettings::narrowphaseMinPairsPerJob`（初始 32）分块经 `JobSystem::ParallelForRange` 派发，主线程参与；worker 只读冻结 Collider/RigidBody/Shape 快照，只写自己的 Pair 输出槽（预分配 `WorkOutput` 数组）与任务私有统计，不改 BVH/map/body、不共享 push_back、不分发事件；chunk 遥测记录槽按 实际chunk数+workerCount 预留（JobSystem 每个参与线程耗尽任务前会多认领一次越界 chunk）；barrier 后主线程按稳定 PairKey 序提交、匹配流形并压缩有效接触。`NarrowPhase::GenerateContact` 统计改为按调用输出参数（删除共享 mutable 成员，这是同实例并发的前提），CCD 栈上局部实例同步适配。**有意行为修正**：旧 `LastQueryStats()` 跨调用差值失败检测有污染 bug——上次失败会使本次正常结果被误判为 QueryFailure、连续失败被漏判冒充正常分离（违反冻结契约）；修正为当次独立判定后 boxfield 轨迹变化（contacts 34883→34235，epaFailures 877→960 恢复真实计数），hull500 全程零失败轨迹位级不变。修复并行 chunk 遥测记录槽越界导致的随机挂起。提供 `SetParallelNarrowphaseEnabled` 与 `SetPhysicsWorkerCount`（workerCount=1 为确定性回退）；`PhysicsSettings.h` 补齐（此前缺失）。验收测试 `Physics_ParallelNarrowphaseTest.cpp` 4 个用例通过（1/N worker 与开关双态位级一致、统计归并等于串行、遥测分块正确），完整 ctest 103/103 通过（1 个 Phase 4 已知禁用）。TSan 在本平台（Windows+MinGW GCC）不可用，以结构保证与确定性测试替代。Release hull500 16 worker narrowPhase P95 2.27→0.52 ms（-77%）、total 3.62→1.79 ms（-51%），远超重复运行波动；Benchmark 新增 `--workers`/`--serial-narrowphase` 与并行遥测输出。详见 `Docs/phase6-parallel-narrowphase.md`。
- 已完成（2026-09-21）：Phase 7 链式前向星与 Physics Island。`PhysicsIsland.h/.cpp` 落地 `PhysicsIslandBuilder`：为参与求解的动态 body 建紧凑节点索引（body id→node），head + `IslandEdge` 连续数组链式前向星；动态-动态非 Trigger 接触写双向边，动态-静态接触归入动态 body 所在岛但静态体不作为传播节点，静态-静态接触不携带岛信息直接跳过；迭代 DFS 提取岛，岛按最小 body id 稳定排序、岛内部 bodies/contacts 升序，`IslandBuildStats` 报告 islandCount/maxIslandBodyCount；builder 复用帧容量（岛数组按索引覆写、跨帧保留 vector 容量），重复构建同输入输出位级一致。`PhysicsWorld::BuildIslands` 每 substep 在求解提交后由 touching 流形构建岛（只读，不改物理状态），`PhysicsStepStats` 新增 `islandCount/islandMaxBodyCount/islandBuildMilliseconds`，`LastIslands()` 暴露最近 substep 快照。Benchmark islands 指标与 islandBuild 分位数接入：boxfield 稳态 86 岛、最大岛 3 body。验收测试 `Physics_PhysicsIslandTest.cpp` 9 个用例通过（链式合并、共享静态地面岛独立、Trigger 不成边、无接触单体岛、稳定排序、容量复用一致、World 集成统计），完整 ctest 112/112 通过（1 个 Phase 4 已知禁用）；boxfield warmup30 轨迹与 Phase 6 基线逐计数一致（contacts 34235、epaFailures 960），同种子双跑 checksum 位级一致。
- 已完成（2026-09-23）：Phase 8 并行 Island Solver。求解输入切换为 Island：`PrepareIslandConstraints` 按岛构建器稳定岛序把 prepared 约束连续排入 `m_preparedConstraints`（岛内保持工作列表 PairKey 升序，Gauss-Seidel 序列与扁平串行逐约束一致）；岛按约束数从大到小稳定排序，`PhysicsSettings::islandSolverMinConstraintsPerJob`（初值 32）以上大岛独立任务、小岛贪心装箱，经 `JobSystem::ParallelForRange` 派发，主线程参与；岛粒度内仍是完整串行 WarmStart→N 次速度迭代→单次位置修正（不做岛内约束着色）；barrier 后主线程按稳定存储序 `CommitSolvedContacts`；`SetParallelIslandSolverEnabled` 与 workerCount=1 为确定性回退，调度结构不变。无数据竞争由结构保证：岛间动态 body 互斥（构建器不变量），共享静态体在求解期纯读取（冲量入口静态保护、位置修正跳过静态端），Debug 构建下每个岛任务以 queryEpoch 票记对本岛动态 body 做原子戳记断言（验收门运行时检查）。`PhysicsStepStats` 新增 islandSolverJobCount/islandSolverWorkerCount/workerBusy/tailWait（口径同窄相：墙钟在 solverMilliseconds，CPU 加和另列）；Benchmark 新增 `--serial-islands`、`config.parallelIslandSolver` 与 `futureMetrics.parallelIslandSolver`。验收测试 `Physics_ParallelIslandSolverTest.cpp` 4 个用例通过（1/N worker 位级一致、开关双态位级一致、共享静态地面双堆叠跨 worker 位级一致、遥测分块与串行回退），完整 ctest 116/116 通过（1 个 Phase 4 已知禁用）；Debug boxfield 三跑（1/8 worker/关岛并行）与 Phase 7 基线逐计数+checksum 位级一致（contacts 34235、epaFailures 960）。Release：stack5×64 独立岛 solver P95 0.410→0.226 ms（-45%）、total -60%；hull500 solver -55%、total -64%；均超过重复运行波动。详见 `Docs/phase8-parallel-island-solver.md`。
- 未开始：Phase 9 及后续睡眠/唤醒与 CCD 新流水线。
- 已记录验证：完整 ctest 116/116 通过（1 个 Phase 4 已知禁用）；相同 seed/body/frame 的两次 boxfield 基准运行中，碰撞计数和最终位置 checksum 一致。

**已落地修改**

- `PhysicsStepStats`：固定步数、检测次数、速度/位置求解次数、静/动态 body 与有效 BVH 叶分类、candidate、Narrowphase、GJK/EPA 调用与失败、manifold/contact point、Integration/Broadphase/Narrowphase/Solver/Total 时间。
- `CollisionDetectionStats` 与 `NarrowPhaseQueryStats`：区分正常分离与 GJK/EPA 算法失败；Narrowphase 内部 AABB 提前拒绝不计为 GJK 调用。
- `PhysicsWorld::SolveContactsIterative()`：默认新路径每 substep Detect 一次、ResolveVelocity N 次、ResolvePosition 一次；`SetLegacyIterativeDetectionEnabled(true)` 保留旧路径用于 A/B。
- `ContactSolver`：新增 `ResolveVelocity()` 与 `ResolvePosition()`，原 `Resolve()` 作为旧路径组合调用；位置修正不再出现在新路径的每轮速度迭代中。
- CCD 临时发布逻辑：`MergeStepContacts()` 合并一个固定步中多个 CCD 子步遇到的唯一 body pair，避免后续空子步清空本步已发生接触。它只影响 `Contacts()` 发布，不会重复求解历史子步接触。
- 外力生命周期（2026-09-07）：`RigidBody::Integrate()` 不再在调用末尾清理外力/力矩；`PhysicsWorld::FixedStep()` 在固定步结束统一 `ClearForces()`。CCD 各子步按自身 dt 消费固定步开始锁定的同一份外力/力矩，瞬时冲量仍只施加一次，一次累计外力只消费于首个实际固定步。
- 根级 `Benchmark/PhysicsBenchmark.cpp`、`Benchmark/CMakeLists.txt`、`Benchmark/README.md`：固定 seed/dt/body/frame，输出 JSON 的 P50/P95/P99、碰撞计数和最终状态 checksum；`Benchmark/` 与 `Tools/` 同级。
- `ThirdParty/CMakeLists.txt`：正式接入仓库内 `nlohmann_json::nlohmann_json` 供 Benchmark 使用。
- `Tests/Physics/Physics_BasicTest.cpp`：新增新流水线单次检测测试与 legacy 重检测测试；修复后完整 Physics 测试通过。
- Phase 1（2026-09-07）：`BvhTree.h/.cpp` 独立单棵 BVH；`BroadPhase.h/.cpp` 定义 `PairKey`/`MakePairKey`/`BroadPhaseQueryCoverage`/`BroadPhaseQueryResult`/`BroadPhaseLeafCounts` 契约并实现 `DynamicBvhBroadPhase`（legacy）与 `HybridBvhBroadPhase`（默认）；`CollisionDetector` 改用查询结果、默认 Hybrid 并在查询后采样实际叶数；`PhysicsWorld` 新增 `SetLegacyBroadPhaseEnabled` A/B 开关；`Tests/Physics/Physics_BroadPhaseTest.cpp` 覆盖全部 Phase 1 验收门。
- Phase 2（2026-09-08）：`Midphase.h/.cpp` 持久 Pair 池与生命周期；`ContactManifold.h/.cpp` 持久缓存字段与确定性锚点匹配；`CollisionDetector` 三阶段拆分与主线程提交；`RigidBody` 外部姿态/结构修订号与 `SetPositionInternal` 内部写入路径；`Collider` 身份/修订号；`PhysicsMaterial::operator==`；`PhysicsWorld` 的 fixedStepId/queryEpoch、接触事件、PairKey 固定步汇总；`Tests/Physics/Physics_MidphaseTest.cpp` 覆盖 Phase 2 验收门。
- Phase 3（2026-09-08）：`RigidBody` 缓存 `m_inverseInertiaTensorWorld` 并统一刷新路径（静态体严格为零）；`ApplyAngularImpulse` 增量式使用缓存；新增 `ApplySolverAngularImpulse` solver 专用入口；`ContactSolver::ApplyImpulsePair` 以有效质量同源矩阵预计算 delta omega 并经该入口写入；`SetAngularVelocity` 经 L 推导保持权威一致；`Tests/Physics/Physics_RigidBodyTest.cpp` 覆盖缓存与公式一致、响应等价、冲量/姿态/积分后 L 与 Iworld*omega 一致、静态切换零缓存、CCD 插值副本无陈旧缓存、非均匀惯量自由旋转（角动量位级守恒，能量漂移实测约 2.8e-2、容差 5e-2，dt=1/120，时长 5 秒）。
- Phase 4（2026-09-08）：`ContactSolver` 重构为 Prepare/WarmStart/SolveVelocityIteration/ResolvePosition/CommitSolvedImpulses 流水线；`ContactManifold` 新增固定步总冲量汇总字段；`PhysicsWorld` 接线子步流水线并删除 legacy 检测开关；`Tests/Physics/Physics_ContactSolverTest.cpp` 覆盖 Phase 4 验收门。
- Phase 5（2026-09-11）：`Runtime/Core/JobSystem.h/.cpp` 固定线程池、批量派发与帧级 barrier；BvhTree 查询回调模板化、`RangeFunction` 非持有 function-ref（热路径去类型擦除）；`Tests/Common/JobSystem_Test.cpp`；方法与数据见 `Docs/defunction-callback-optimization.md`。
- Phase 6（2026-09-12）：`CollisionDetector::ExecuteNarrowphase` 并行分块 + 私有输出槽 + 主线程稳定顺序提交；`NarrowPhase` 按调用输出统计（删除共享 mutable 成员，修正跨调用失败检测污染）；`PhysicsSettings.h` worker/分块参数；`PhysicsWorld` 并行开关与 worker 数；`Tests/Physics/Physics_ParallelNarrowphaseTest.cpp`；Benchmark `--workers`/`--serial-narrowphase` 与并行遥测；详见 `Docs/phase6-parallel-narrowphase.md`。
- Phase 7（2026-09-21）：`PhysicsIsland.h/.cpp` 链式前向星岛构建器（紧凑动态索引、双向动态边、静态不成节点、稳定排序、帧容量复用）；`PhysicsWorld::BuildIslands` 每 substep 只读接线与 `PhysicsStepStats` 岛统计；Benchmark islands 指标与 islandBuild 分位数；`Tests/Physics/Physics_PhysicsIslandTest.cpp`。
- Phase 8（2026-09-23）：`PhysicsWorld` 求解输入切换为 Island——`PrepareIslandConstraints`（按岛分组连续 prepared 约束）+ `SolveIslandConstraints`（大岛独立任务/小岛装箱、并行派发或串行内联、岛粒度完整 WarmStart/速度迭代/位置修正）+ 主线程稳定序提交；`PhysicsSettings.h` 新增 `parallelIslandSolverEnabled` 与 `islandSolverMinConstraintsPerJob`；`PhysicsStepStats` 岛求解器遥测；Benchmark `--serial-islands` 与 `futureMetrics.parallelIslandSolver`；`Tests/Physics/Physics_ParallelIslandSolverTest.cpp`；详见 `Docs/phase8-parallel-island-solver.md`。

**近期补丁清单（进入双 BVH 前）**

1. **已完成（2026-09-07）。** 修正 `PhysicsWorld.cpp` 中 `MergeStepContacts(stepContacts, m_contacts);` 的缩进；仅格式问题。
2. **已完成（2026-09-07）。** 为 `MergeStepContacts()` 增加明确注释，说明它是 CCD 固定步发布层的临时去重，不是 Midphase，也不承担跨帧 Pair 生命周期。
3. **已完成（2026-09-08，提前于 Phase 4）。** `MergeStepContacts()` 已删除，由 `PhysicsWorld` 内 PairKey→索引映射的固定步接触汇总替代 O(n^2) 线性查找；持久 Pair 池负责身份和缓存。发布语义已验证：本步曾接触汇总、步内 Enter 后 Exit（含 CCD 瞬态 trigger 回归 `CcdTriggerEnterAndExitInsideSingleFixedStep`）。
4. **已完成（2026-09-08）。** `SolveContactsIterative()` 已更名为 `DetectAndSolveContacts(substepDt, isToiSubstep)`，随 Phase 2 的 PhysicsWorld 接线一起完成。
5. **已完成（2026-09-07）。** 基准程序新增 `--scenario` 枚举：`boxfield`（原固定种子 box workload，行为与 RNG 顺序不变）、`stack3`/`stack5`（3/5 箱堆叠）、`hull`（100/500/1000 凸包，经 `--bodies` 参数化，32 顶点固定种子凸包）、`trigger`、`churn`（每帧 5% 创建销毁）、`ccd`（250 m/s 高速球持续冲击，零弹性地面 + 沉降重发保证每帧 CCD 活跃）、`all`（一次输出全场景基线）。新增 `--warmup` 预热帧参数。JSON 升级 schemaVersion 2，增加 version 块（项目版本、编译器、构建类型、git commit/dirty、UTC 时间戳）。已生成 `Build/physics_benchmark_baseline.json`（全场景）与 `Build/physics_benchmark_hull500.json`；boxfield 同种子双跑 checksum 位级一致。注意：该基线产自 Debug 构建，正式性能验收按计划使用 Release。
6. **已完成（2026-09-07）。** `staticBvhLeafCount/dynamicBvhLeafCount` 现在在查询同步后读取两棵实际 BVH 的叶数（`HybridBvhBroadPhase::GetLeafCounts()`）；legacy 单树路径无分类树，回退为逻辑分类基线。顺带修复了在 `ComputePairs` 之前采样导致首帧统计恒为 0 的问题。
7. **已完成（2026-09-08）。** Midphase 指标已接入 Benchmark：steady-state `activePairs/newPairs/removedPairs` 真实数据替换 `available: false`；boxfield 稳态 240 帧 newPairs=0、removedPairs=0。Island 指标已随 Phase 7 接入（2026-09-21），Sleep 仍为占位。
8. **已完成（2026-09-08）。** `normalImpulse` 已升级为最近有效接触子步内实际施加的累计冲量，Trigger 所有冲量字段恒零并有回归，GameApp 遥测已按字段口径加注释。切向冲量累计（双切线基投影）与固定步总冲量汇总字段已随 Phase 4 落地：`ContactManifold` 新增 `fixedStepNormalImpulse`/`fixedStepTangentImpulse`，由 `PhysicsWorld` 在发布时按子步最终累计 lambda 合计（WarmStart 与后续 delta 的合计），Trigger 恒零。
9. **已完成（2026-09-07）。** 跨阶段契约已冻结（见下文）。已为 CCD 外力跨子步清理问题增加聚焦回归 `CcdSubStepsConsumeForceAcrossFullFixedStep` 并完成修复：旧行为下外力仅在首个 TOI 子步按子步 dt 消费后被清空（回归实测 vx≈11.7 而非 50），修复后各子步按自身 dt 积分同一份锁定外力、固定步末统一清理。修复后完整 Physics 测试 22/22 通过。

**跨阶段契约（先于 Phase 1 冻结）**

- **时间与 Pair 生命周期。** `fixedStepId` 标识固定步，`queryEpoch` 标识每次离散或 TOI 子步查询；查询必须说明覆盖范围。Broadphase 未返回 Pair 只有在该 Pair 属于本次完整重检范围时才可作为移除依据；因睡眠跳过查询不代表分离。区分候选 Pair 生命周期与真实 touching 状态：fat AABB 仍重叠但 Narrowphase 分离时，保留候选 Pair、清空接触缓存并产生接触 Exit。算法失败不冒充正常分离，必须单独计数并明确采用的接触发布策略。
- **写入来源与外力生命周期。** 外部 Force/Torque/Impulse、速度/位姿及结构修改触发 wake request；积分、WarmStart、Solver 冲量和位置修正使用内部写入路径，不重置睡眠计时，但仍维护姿态缓存及 BVH 脏标记。固定步开始锁定本步外力/力矩，各子步按自己的 `dt` 积分，固定步结束统一清理；外部瞬时冲量只施加一次。一个 `Step()` 执行多个固定步时，一次累计外力只消费于首个实际固定步，持续力由固定步驱动方每步提交；未执行固定步不得消费外力。
- **缓存含义。** lambda 是当前 substep 约束的累计解，旧值只作 WarmStart 初值。持久数据额外保存旧切线基或世界空间摩擦冲量、`cachedDt` 及是否属于 TOI 冲击；新基中重投影摩擦冲量并按新摩擦上限夹紧。普通持续接触按 `dtNew / cachedDt` 缩放初值；首版 TOI 冲击接触不跨子步 WarmStart，未受冲击的持续接触可按前述规则复用。TOI 结束后冲击缓存无效，不混入下一固定步。
- **发布与事件。** 持久 Pair 状态、步末接触快照、固定步曾接触汇总、步内事件记录相互独立。为保持当前行为，`Contacts()` 返回最近一个已完成固定步中曾接触的唯一 Pair，几何与 `normalImpulse` 取最后一次有效接触子步的结果；不宣称它是步末仍接触集合，也不用于驱动下一步求解。一次 `Step()` 内事件覆盖全部实际固定步；Enter/Exit 按 `(fixedStepId, queryEpoch, PairKey)` 发布，同一固定步短暂接触允许 Enter 后 Exit，Stay 每个持续接触 Pair 每固定步至多一次。睡眠不触发 Exit；销毁/过滤失效产生带稳定 ID 的 Exit，不保留悬空引用。
- **遥测。** `normalImpulse` 明确为最后有效接触子步内累计 lambda；固定步总冲量若需要，另设汇总字段，统计各子步实际施加的净冲量（WarmStart 与后续 delta 的合计），不能逐迭代累加累计 lambda。Trigger 所有冲量字段恒零。GameApp 必须按字段口径展示，不能把最后子步值标作固定步总量。
- **并发结构冻结。** 从任务派发到 barrier，World 不允许创建/销毁 body、替换 Collider、改变 Shape 或外部写入刚体状态。首版通过同步调用契约收口，不要求立即引入 command buffer；事件回调只在主线程提交完成后执行。连续 slot 在任务运行期间不得扩容、回收或复用，删除发生在同步点。

**修改与失效矩阵**

| 修改来源 | BVH / Pair 操作 | 冲量缓存 | 唤醒 |
| --- | --- | --- | --- |
| 内部积分、位置修正 | 更新 transform version 与动态叶脏标记；按查询范围重检 | 通过锚点/法线匹配复用，不因正常运动全部清零 | 不作为外部活动 |
| 外部位姿修改（首版按 teleport） | 更新叶并重检关联 Pair | 清空关联缓存 | 本体及受影响接触岛 |
| 创建、销毁、Collider/Shape 替换、静动态切换 | 插入/移除/迁移叶并使关联 Pair 失效 | 清空；销毁先生成稳定 ID 事件 | 旧邻接岛及新接触岛；包括移走/删除静态支撑 |
| layer/mask 修改 | 不重建几何叶；重新过滤旧 Pair 并查询可能新增 Pair | 被过滤接触清空 | 受影响接触岛 |
| Trigger、单面开关/法线修改 | 重筛关联接触并更新事件状态 | 清空关联缓存 | 受影响接触岛 |
| 材质修改 | 不更新几何叶；重新 Prepare | 首版清空关联缓存 | 关联接触岛 |
| 质量/惯量修改 | 不更新几何叶；刷新所需惯量缓存和约束响应 | 清空关联缓存 | 本体所在岛 |
| 外部力/力矩/冲量、速度修改 | 下次查询使用新运动状态；不无故重建静态树 | 按接触匹配规则处理 | 本体所在岛 |

所有相关 setter 必须递增对应 revision；不能假设 World 可变指针天然携带修改通知。首版保留可变 `Material()` 时在同步阶段比较材质值；Shape 是否允许原地修改必须冻结为明确契约：首版共享 Shape 在仿真期间不可变，几何变化通过替换 Shape 进入版本路径。Collider 身份替换必须可辨识，不能因新对象 revision 恰好相同漏掉失效。失效矩阵由 Agent 3/4/5/7 共同确认接口，后续 Agent 12 复用，不重新定义。

**Steps**

### Phase 0：基线、契约与流水线纠正
1. **部分完成。** Agent 0 已建立固定 seed/dt/body/frame 的 box workload、统计结构、JSON P50/P95/P99、checksum 和重复运行确定性检查；已统计逻辑 Static/Dynamic 叶分类、candidate/manifold/contact、GJK/EPA 调用与失败和已有阶段耗时。2026-09-07 补丁 5 已补齐场景矩阵（stack3/stack5、hull 100/500、trigger、churn、ccd）与带版本信息的基线 JSON；hull 1000 与 Release 正式基线、质量阈值登记留待 Phase 11。Island 指标已随 Phase 7 接入；awake/sleeping 指标等待 Phase 9。
2. **已完成。** Agent 1 已重构 PhysicsWorld：每个普通 substep 只执行一次 Detect，之后对同一批约束执行 N 次速度求解，substep 末执行一次位置修正；CCD 每个实际 TOI substep检测一次；已增加检测调用计数、legacy A/B 开关和测试。
3. **部分完成。** 现有 Broadphase 已按 body ID 规范化 pair，Body ID 当前不复用；先冻结上述跨阶段契约并补 CCD 外力/力矩生命周期回归。累计 normal/tangent impulse、Trigger Pair 生命周期和 GameApp 遥测由 Midphase/Warm Start 阶段实现，不提前改变字段含义。

### Phase 1：双 BVH Broadphase
4. **已完成（2026-09-07）。** Agent 2 从 DynamicBvhBroadPhase 提取单棵 BvhTree（独立文件 `BvhTree.h/.cpp`），新增 HybridBvhBroadPhase 管理 static/dynamic 两棵树。动态树包含所有非静态 body（Phase 9 前全部视为 active）；静态树仅在其 tight AABB 离开已存 fat AABB 时重插，即创建、销毁、Collider/Shape/Transform 变化或静动态切换时更新。
5. **已完成（2026-09-07）。** Agent 2 实现 Dynamic-Dynamic 自查询与 Dynamic-Static 查询，不生成 Static-Static；输出规范化、去重、稳定排序的 PairKey 及查询覆盖范围（首版 `FullScene`）。保留旧 Broadphase A/B 路径（`SetLegacyBroadPhaseEnabled`），以相同冻结输入比较结果。
6. **已完成（2026-09-07）。** 验收门全部通过并建有回归：静态物体移动/替换 Collider 后树正确失效；动态/静态切换不残留叶；共享静态地面不产生重复 Pair；销毁 body 无残留叶；相同叶 AABB/过滤条件下与 legacy 候选集严格一致，小规模暴力 AABB oracle 验证无漏检，相同 Narrowphase 下双路径最终接触集合与状态一致（`Physics_BroadPhaseTest.cpp`）。

### Phase 2：完整 Midphase 与持久流形
7. **已完成（2026-09-08）。** Agent 3 新增 Midphase.h/.cpp：unordered_map<PairKey, PairHandle> + 连续 slot/free-list 存储 MidphasePair；维护 New/Persisting/Removed、fixedStepId、lastSeenQueryEpoch、Collider 身份/revision、hadContact、needsNarrowphase、持久 ContactManifold 和连续 narrowphase work list。候选移除与 touching 分离分别处理（分离保候选清缓存发 Exit，FullScene 查询未返回才移除候选），睡眠跳过不删除 Pair（仅 FullScene 覆盖范围可作为移除依据）。工作列表为连续 slot 索引；Removed slot 延迟到下一次 BeginQuery 同步点回收并递增代数，同步点后旧句柄失效。
8. **已完成（2026-09-08）。** Agent 3 扩展 ContactPoint：localPointA/localPointB、accumulatedNormalImpulse、glm::vec2 accumulatedTangentImpulse，并保存旧切线基（cachedTangent1/2）及 cachedDt/TOI 缓存标记。第一版使用两侧 local anchor 距离（阈值 0.05）与法线阈值（cos30°）做确定性一对一匹配，距离并列时取最小旧索引；点顺序变化不串错冲量（`Physics_MidphaseTest.cpp` 重排/并列/超阈值/法线翻转用例）。
9. **已完成（2026-09-08）。** Agent 4 拆分 CollisionDetector：Broadphase 提交 PairKey 和查询覆盖范围；Midphase 更新 Pair 生命周期并构建工作列表；Narrowphase 为每个槽生成新几何流形；主线程提交结果、匹配旧点并转移缓存。接触分离、法线显著翻转及失效矩阵规定的修改（Collider 身份/revision、材质值、外部姿态、质量/惯量/静态切换）清零缓存；正常运动（内部积分/位置修正）不清缓存。摩擦缓存的旧切线基已持久化，Phase 4 的 Prepare 负责重投影到新基，禁止直接把旧二维分量用于新基。
10. **已完成（2026-09-08）。** DestroyRigidBody 经下一次 FullScene 查询扫描移除 Pair 并发布带稳定 PairKey 的 Exit（无悬空引用）；AttachCollider 分配新 Collider 身份使替换可辨识；RemoveCollider/layer/mask/材质/Trigger/one-sided 变更均经绑定快照比较在同步点失效；`RigidBody` 外部 SetPosition/SetOrientation（teleport）与内部 `SetPositionInternal`/积分写入可区分。单线程阶段结构修改天然在冻结区间之外；command buffer 留作后续收口。
11. **已完成（2026-09-08）。** 验收门通过（`Physics_MidphaseTest.cpp`，19 个新用例）：候选与 touching 生命周期正确；销毁/替换无悬空 Pair；稳态 Pair 无新增/移除（基准稳态 240 帧 newPairs=0）；Trigger Enter/Stay/Exit 不重复不丢失，一个固定步内 Enter 后 Exit 与单次 Step 多固定步的事件覆盖均有回归；单线程 Hybrid 与 legacy Broadphase 接触集合一致。睡眠跳过查询的验收待 Phase 9 引入非 FullScene 覆盖范围后补充。

### Phase 3：刚体角状态与预计算响应
12. **已完成（2026-09-08）。** Agent 5 在 RigidBody 缓存 world inverse inertia，统一刷新构造、SetOrientation、SetInertiaTensorDiagonal、SetStatic 和 Integrate 姿态更新路径。静态体缓存严格为零。SetMass 仅在未来联动惯量时触发刷新（当前不刷新，已在访问器注释冻结）。
13. **已完成（2026-09-08）。** Agent 5 保留 m_angularMomentum 作为本地权威状态，避免非均匀惯量刚体自由旋转退化。普通 ApplyAngularImpulse 使用缓存执行 L += angularImpulse 与 omega += Iworld^-1 * angularImpulse；姿态/惯量变化后由 L 重新推导 omega，包括 Integrate 更新姿态后。新增 solver 专用“已计算角响应”入口 `ApplySolverAngularImpulse`，一次调用同时累加 angular impulse 到 L、累加预计算 delta omega 到 omega；`ContactSolver::ApplyImpulsePair` 已接入该入口，求解器不再直接写 omega 导致 L/omega 分叉。内部响应和位置修正不发外部 wake request（沿用 SetPositionInternal 与零 revision 写入）；公开修改入口仍按失效矩阵工作。
14. **已完成（2026-09-08，契约冻结）。** 网络边界只定契约、不实现协议：未来快照传 orientation + angularVelocity，应用顺序为 SetOrientation 后 SetAngularVelocity；惯量由资源版本保证一致。契约已写入 `RigidBody.h` 的 `SetAngularVelocity` 注释；StateSnapshot/ClientPredictor 仍是占位，本阶段未修改网络消息。
15. **已完成（2026-09-08）。** 验收门：`Physics_RigidBodyTest.cpp` 验证 cached Iworld^-1 与 R * Ibody^-1 * R^T 一致；预计算响应与普通 ApplyAngularImpulse 等价；固定姿态多次冲量及积分姿态更新后 L 与 Iworld * omega 一致（无扭矩时 L 位级不变）；静态切换缓存严格为零、CCD 插值副本无陈旧缓存；非均匀惯量自由旋转角动量位级守恒，能量漂移实测约 2.8e-2、登记容差 5e-2（dt=1/120，时长 5 秒，一阶半隐式姿态积分）。

### Phase 4：约束准备、Warm Start 与累计冲量
16. **已完成（2026-09-08）。** Agent 6 在 ContactSolver 引入 substep 临时 PreparedContactConstraint/PreparedContactPoint，与持久 ContactPoint 分离。Prepare 阶段计算 rA/rB、稳定正交双切线基、ra/rb cross axis、单位 delta lambda 对两体的 delta v/delta omega、法向/双切向有效质量和 restitution bias。首版 penetration velocity bias 为零，穿透仅由位置修正处理。这些响应只在当前 substep 有效，不跨姿态变化持久化。
17. **已完成（2026-09-08）。** Agent 6 增加 WarmStart：按缓存契约进行 dt 缩放、切线基重投影和夹紧后，每 substep 仅施加一次初值；TOI 冲击接触（当前或缓存来源）首版初值为零。一个 delta lambda 作为同一约束更新，同时修改线速度、角速度和内部 L（经 `ApplySolverAngularImpulse`），不发外部 wake request。
18. **已完成（2026-09-08）。** Agent 6 将速度求解改为累计增量形式：每轮计算 delta lambda；累计法向冲量夹紧到 lambda_n >= 0；累计双切向冲量投影到半径 mu * lambda_n 的摩擦圆盘；只施加新旧累计值之差，允许负 delta 撤回过大的 WarmStart 初值。Restitution 取 WarmStart 前入射速度，只进入 Prepare 的速度 bias，并设低速反弹阈值（1.0 m/s），避免每轮重复反弹。
19. **已完成（2026-09-08）。** Agent 6 保留 one-sided 法线朝向和正反面判定并建立独立回归（`OneSidedPlatformBouncesOnceWithoutBoost`、`OneSidedPlatformPassesThroughFromBack`）；新路径有效质量匹配实际施加响应（`BuildAxis` 预计算与分母共享逆惯量矩阵），不照搬旧实现仅缩放分母角项的做法。首版采用完整物理角响应，旧 bounce boost 的反弹需求由 Prepare 中一次性目标速度表达，不额外改速度。SolvePosition 每 substep 一次，沿用 slop/ratio，通过堆叠场景调参，不重跑碰撞检测。
20. **已完成（2026-09-08）。** Agent 7 作为唯一 PhysicsWorld 集成负责人：固定步开始锁定外力；每 substep 接为 Integrate -> 同步修改/Broadphase -> Midphase -> Narrowphase once -> Prepare -> WarmStart once -> SolveVelocity N 次 -> SolvePosition once -> CommitSolvedImpulses -> 更新接触/事件记录；固定步末清理外力并发布汇总。`SetLegacyIterativeDetectionEnabled` 迁移路径已删除（Phase 0 A/B 工具收尾）。normalImpulse 是最后有效接触子步的累计值，fixedStepNormalImpulse 是跨子步净冲量合计。
21. **部分完成（2026-09-08）。** 验收门：warm start 后迭代只应用 delta lambda、过大初值可撤回（`AppliesCachedImpulseOnceAndRetractsOverestimate`）；lambda_n 非负且 ||lambda_t|| <= mu*lambda_n（`FrictionDiscClampHoldsEveryIteration`）；流形重排、切线基切换仍继承正确冲量（`ReorderedManifoldInheritsImpulsesByAnchor`、`ReprojectsCachedTangentImpulseOntoNewBasis`）；变长子步缩放符合契约（`ScalesCacheBySubstepDtRatio`），TOI 冲击不重放（`ToiImpactContactsNeverReplayCache`）；有效质量与实际速度响应一致（`EffectiveMassMatchesActualVelocityResponse`）；one-sided 无额外重复反弹（`OneSidedPlatformBouncesOnceWithoutBoost`）。**低 iteration 堆叠质量测试（`LowIterationStackStaysWithinQualityBounds`）暂时禁用**：新求解器在 4 次迭代下穿透 0.011（优于旧实现 0.171）且速度收敛，但位置在 300 步后漂移坍塌；根因是多点流形 + warm start + 顺序冲量的架构限制（零臂质心点作为负载汇点，无法在倾斜时重新分配负载到角点），需后续块求解器或稳定 feature ID 解决。详见 `Docs/phase4-investigation.md`。

### Phase 5：固定 JobSystem
22. **已完成（2026-09-11）。** Agent 8 新增 Runtime/Core/JobSystem（`JobSystem.h/.cpp`），使用固定 std::jthread worker、条件变量、批量任务（每 chunk 一张 ticket）与帧级 barrier；支持主线程参与、workerCount=1、空任务、重复初始化与安全关闭。无每帧 std::async。
23. **已完成（2026-09-11）。** 提供 `ParallelForRange(count, minItemsPerJob, function)` 连续区间接口；任务异常被捕获并计数，barrier 不会永久等待，首个异常在调用线程重抛。该阶段未修改物理算法。
24. **已完成（2026-09-11）。** 验收门通过（`Tests/Common/JobSystem_Test.cpp`，9 个用例）：每个索引恰执行一次；1/2/4/硬件并发数结果一致；关闭时无死锁；小任务退化为调用线程直接执行。完整 ctest 99/99 通过（1 个 Phase 4 已知禁用）。
25. **已完成（2026-09-11，热路径回调去类型擦除）。** 物理热路径仅有两处 `std::function`：`BvhTree::SelfQueryPairs/QueryPairsAgainst` 的 per-pair 回调（每帧数千次调用）与 `JobSystem::RangeFunction`（每 substep 一次）。裸函数指针无法携带捕获状态且调用开销与 `std::function` 相同（同为间接跳转），故不采用 Engine 式函数指针，改为：BvhTree 两个查询改为模板成员函数（回调内联进查询循环，消除间接调用）；`RangeFunction` 改为 16 字节非持有 function-ref（data 指针 + invoke thunk），生命周期由 `ParallelForRange` 的 barrier 语义保证，彻底消除闭包堆分配。语义零变化：candidates 106080 与接触点数 34883 均与 Phase 4 基线一致，双跑 checksum 位级一致，完整 ctest 99/99 通过。单变量 Release A/B（仅切换 BvhTree 回调机制，其余不变，5 次采样取 P95 中位数，candidates/contacts/checksum 位级一致）：boxfield total 0.839→0.777 ms（-7.4%）、broadPhase -11%；hull500 total 3.892→3.261 ms（-16%）、broadPhase -23%、narrowPhase 连带 -20%。Debug 下几乎无差异（本就不内联），正式验收以 Release 为准。方法、数据与结论详见 `Docs/phase5-callback-optimization.md`。

### Phase 6：并行 Narrowphase
25. **已完成（2026-09-12）。** Agent 9 基于 Midphase 连续 work list 分块，minPairsPerJob=32，主线程参与；任务数由块大小与实测负载调整，不硬性限制为 worker 数，以允许成本不同的凸包查询负载均衡。Worker 只读冻结的 Collider/RigidBody/Shape，只写自己的 Pair 输出槽；未修改 BVH/map/body、无共享 push_back 和事件分发，查询统计使用任务私有计数并在 barrier 后归并。
26. **已完成（2026-09-12）。** Barrier 后主线程按 PairKey 稳定顺序提交结果、匹配流形并压缩有效接触。提供 `SetParallelNarrowphaseEnabled` 与 `SetPhysicsWorkerCount`；workerCount=1 为确定性回退。
27. **已完成（2026-09-12）。** 验收门：1 worker 与 N worker 的 Pair、contact point、法线、事件与最终状态位级一致；本平台无 TSan（已记录），以结构保证与确定性测试替代；统计每线程 job 数、耗时和尾部等待并随 Benchmark 输出；500 凸包相对单线程 narrowPhase -77%、total -51%，超过重复运行波动。详见 `Docs/phase6-parallel-narrowphase.md`。

### Phase 7：链式前向星与 Physics Island
28. **已完成（2026-09-21）。** Agent 10 新增 PhysicsIslandBuilder（`PhysicsIsland.h/.cpp`）：仅为当前实际参与求解的动态 body 建立紧凑索引，head + `IslandEdge` 连续数组链式前向星。动态-动态非 Trigger 接触写双向边；动态-静态约束归入动态 body，静态体不作为传播节点。未引入运动学 body。
29. **已完成（2026-09-21）。** 迭代 DFS 提取稳定排序的 PhysicsIsland（岛按最小 body id、岛内部 bodies/contacts 升序），复用帧容量；初版使用 vector，offset/count 帧分配器留待稳定后。睡眠岛成员保留规则待 Phase 9 引入睡眠状态后接线（当前所有动态 body 均参与建岛）。
30. **已完成（2026-09-21）。** 验收门通过（`Physics_PhysicsIslandTest.cpp`，9 个用例）：每个动态 body 最多属于一个 Island；多个 body 共享静态地面仍为独立 Island；Trigger 不成边；无接触的动态 body 形成单体 Island；重复构建容量复用输出一致；World 集成后统计快照正确（boxfield 稳态 86 岛、最大 3 body）。复杂度 O(V+E)，接触按边一次性写链、body 按 DFS 各访问一次。

### Phase 8：并行 Island Solver
31. **已完成（2026-09-23）。** Agent 11 将 Prepare/WarmStart/SolveVelocity/SolvePosition 输入切换为 Island。Island 内保持串行 Sequential Impulse；不同 Island 任务独占动态 body。按约束数量从大到小排序，大 Island 单独任务，小 Island 批量装箱降低尾部等待（`islandSolverMinConstraintsPerJob`，初值 32）。
32. **已完成（2026-09-23）。** 稳定顺序不由任务完成顺序决定：岛内约束序与岛提交序均在主线程调度时固定，事件与 Contacts 在 barrier 后由主线程按稳定键发布。未做同一 Island 内约束图着色。
33. **已完成（2026-09-23）。** 验收门通过（`Physics_ParallelIslandSolverTest.cpp`）：Debug 构建运行时断言不同任务不写同一动态 body（queryEpoch 票记戳记，全程未触发）；1 worker 与 N worker 接触集合与最终状态位级一致（强于容差要求）；Release stack5 场景 64 个独立堆叠随 worker 数扩展（total P95 -60%@w8，超重复运行波动）；单个巨大 Island 不承诺线性加速（未包含在测试矩阵）。

### Phase 9：Island 睡眠与唤醒
34. Agent 12 为 RigidBody 增加 Awake/Candidate/Sleeping、sleepTimer、allowSleep、transformVersion、externalActivity 标记。按已冻结失效矩阵接入外部 wake request；内部积分、WarmStart、Solver 冲量/位置修正及正常重力不标记为外部活动。静态体本身无需唤醒，但移走、销毁或修改静态支撑必须通知其旧接触邻接岛。
35. 在 Island 级判定连续静止：所有动态 body 允许睡眠，同时满足线速度平方、角速度平方阈值，无外部活动且接触稳定，按实际模拟 dt 累计 timeToSleep 后整岛睡眠。初始参数 0.05 m/s、0.05 rad/s、0.5 s，放入 PhysicsSettings。睡眠时清零残余速度与角动量、同步角状态；保留 Pair 和邻接，不产生接触 Exit。
36. Sleeping body 保留在 dynamic BVH，但跳过积分、主动查询、sleeping-sleeping Narrowphase 和 solver。活动动态体 Broadphase 命中 sleeping body 时，在当前 substep 求解前完成整岛唤醒；对新活动成员补齐必要查询并去重，唤醒传播达到闭包后再构建本子步 Narrowphase/Island 工作列表。按本子步 dt 为新唤醒 body 补上尚未消费的外力/重力速度增量且仅一次，未来子步正常积分；不得重复推进已完成积分的 body。睡眠未查询 Pair 保留，静态修改查询需覆盖 sleeping body。
37. 验收门：支撑冲量和位置修正不会阻止稳定堆叠最终睡眠；高速命中在同一子步求解前完整唤醒；外力、冲量、显式速度/位姿、Collider 替换均唤醒；移走/删除地面可唤醒原支撑岛；睡眠期间不误发 Exit/Enter，唤醒后不丢接触或重复消费外力。记录睡眠耗时和最大穿透。

### Phase 10：CCD 接入新流水线
38. Agent 13 用保守 swept AABB 查询双 BVH，对启用 CCD 且运动超过阈值的活动动态体生成 CCD Pair；动态目标的运动也必须纳入查询边界，高速旋转凸包不能仅看质心位移。明确采用的旋转扫掠界和相对运动判定，不恢复全 Pair 固定采样。CCD Pair 复用 Midphase 身份，每个 TOI substep 重新 Prepare，冲击缓存按既定策略失效；TOI 搜索与离散 Narrowphase 调用分别统计。
39. 首版保留全局 TOI 子步，只替换候选生成和新流水线接线；离散/CCD Pair 按 PairKey 去重，但不同 queryEpoch 的几何必须重新求值。明确零 TOI 最小推进、最大子步预算及预算耗尽策略，并计数暴露；不得把无保护的剩余时间整段推进视为已满足防穿透验收。Island-local CCD 拆为独立后续里程碑，不阻塞本轮性能验收。
40. 验收门：高速球/凸包、双方高速相向运动及高速旋转长凸包场景无漏候选和穿透；相同外力/力矩、总 dt 下不同子步划分的速度增量符合数值容差；冲击不跨子步重复 WarmStart；短暂接触事件不被步末分离吞掉；零 TOI/预算耗尽行为符合冻结策略。CCD 开关保持兼容，SpinningSphere 与 CcdPreventsHighSpeedTunneling 回归通过。候选量同时受高速 body 数和局部密度影响，不承诺只与高速 body 数成正比。

### 后续独立里程碑：Island-local CCD（不计入本轮完成条件）
- 在 Phase 10 全局 TOI 路径稳定后单独设计：跨岛碰撞与岛合并、各岛时间一致性、运动目标预测、外力消费、睡眠唤醒和事件顺序。明确哪些岛需要共同推进，不能直接把全局循环改成只推进命中的 body。
- 以全局 TOI 路径为参考，新增跨岛高速碰撞、TOI 期间岛合并和不同推进顺序的对照测试后，再评估局部子步收益与上线条件。

### Phase 11：最终回归与性能验收
41. Agent 14 执行完整 Physics_Test、线程数 1/2/4/硬件并发、固定随机种子重复运行、高频创建销毁、静动态切换和 10 分钟 100/500/1000 凸包稳定性测试；检查 NaN、爆速、穿透、事件一致性。状态比较按 body ID 排序，覆盖位置、姿态、线速度和角速度；相同构建固定输入检查可重复性，跨线程结果使用事先记录的容差，不承诺跨平台位级一致。
42. 输出阶段对比：Broadphase/Midphase/Narrowphase/Island build/Solver/total 的墙钟 P50/P95/P99，另列 worker CPU 时间总和，禁止二者混加。记录 Pair cache 命中率、稳态分配次数/字节、job 尾部等待、awake/sleeping 数及物理质量。目标为稳态 Pair 处理相关分配次数较基线减少 90% 以上（包含 map 节点、slot 扩容、work list 与临时容器，并同时报告字节数），500 凸包并行 Narrowphase 收益超过重复运行波动，独立 Island solver 可扩展，睡眠后 total ms 明显下降。比较必须保持相同输入和质量门槛，不能以漏接触或额外穿透换取加速。

**Relevant files**
- `e:/MyGame/Runtime/include/Physics/BvhTree.h`、`e:/MyGame/Runtime/src/Physics/BvhTree.cpp` — 单棵增量 BVH（Phase 1 起独立文件）。
- `e:/MyGame/Runtime/include/Physics/BroadPhase.h`、`e:/MyGame/Runtime/src/Physics/BroadPhase.cpp` — PairKey/查询覆盖范围契约、DynamicBvhBroadPhase（legacy A/B）与 HybridBvhBroadPhase。
- `e:/MyGame/Runtime/include/Physics/Midphase.h`、`e:/MyGame/Runtime/src/Physics/Midphase.cpp` — Pair 生命周期、稳定池、持久流形和工作列表。
- `e:/MyGame/Runtime/include/Physics/ContactManifold.h` — local anchors 与累计法向/双切向冲量。
- `e:/MyGame/Runtime/include/Physics/CollisionDetector.h`、`e:/MyGame/Runtime/src/Physics/CollisionDetector.cpp` — 拆分 Broadphase/Midphase/Narrowphase 阶段。
- `e:/MyGame/Runtime/include/Physics/RigidBody.h`、`e:/MyGame/Runtime/src/Physics/RigidBody.cpp` — 缓存世界逆惯量、L/omega 一致性和预计算角响应入口。
- `e:/MyGame/Runtime/include/Physics/ContactSolver.h`、`e:/MyGame/Runtime/src/Physics/ContactSolver.cpp` — Prepare/WarmStart/SolveVelocity/SolvePosition。
- `e:/MyGame/Runtime/include/Physics/PhysicsIsland.h`、`e:/MyGame/Runtime/src/Physics/PhysicsIsland.cpp` — 链式前向星与 Island 提取。
- `e:/MyGame/Runtime/include/Physics/PhysicsSettings.h` — worker、分块、solver 和睡眠参数。
- `e:/MyGame/Runtime/include/Core/JobSystem.h`、`e:/MyGame/Runtime/src/Core/JobSystem.cpp` — 固定线程池。
- `e:/MyGame/Runtime/include/Physics/PhysicsWorld.h`、`e:/MyGame/Runtime/src/Physics/PhysicsWorld.cpp` — 阶段编排、生命周期、睡眠和事件提交。
- `e:/MyGame/Runtime/src/Physics/ContinuousCollision.cpp` — swept query 与 TOI Island。
- `e:/MyGame/Tests/Physics/Physics_BasicTest.cpp`、`e:/MyGame/Sample/PhysicsCollisionSample.cpp` — 正确性、稳定性和性能场景。
- `e:/MyGame/Benchmark/PhysicsBenchmark.cpp`、`e:/MyGame/Benchmark/README.md` — 当前固定种子无窗口基准及 JSON 使用说明。
- `e:/MyGame/Game/src/Core/GameApp.cpp` — 累计 normalImpulse 遥测语义。

**Verification**
1. 每个实现 Phase 必须先跑其新增聚焦 GTest，再跑完整 Physics_Test；并保存同输入的旧路径 A/B 差异。纯文档修订只检查文档一致性，不声称重新通过实现测试。涉及有意算法改变时比较约定不变量和质量指标，而不是强求旧轨迹完全一致。
2. 核心现有测试至少包括 AngularVelocityUpdatesOrientation、FaceContactBuildsFourPointManifold、SpinOnGroundGeneratesSidewaysVelocity、SpinningSphereDoesNotSinkBelowGroundTop、CcdPreventsHighSpeedTunneling。
3. 数学测试覆盖 cached Iworld^-1、预计算 delta omega、姿态更新后的 L/omega 一致性、自由旋转角动量守恒和能量漂移、有效质量/实际响应一致、负 delta 撤回、切线基转换、累计 lambda 夹紧与摩擦圆盘、变长子步缓存缩放及外力/力矩消费。
4. 生命周期测试覆盖候选 Pair 与 touching 的不同生命周期、流形点重排、法线翻转、失效矩阵各入口、睡眠跳过查询不移除、支撑移除唤醒、Body 销毁，以及单个固定步和单次 Step 多固定步中的 Trigger enter/stay/exit 顺序。固定步曾接触汇总不能替代事件记录。
5. 并发测试覆盖冻结区间结构修改约束、work slot 独占和统计归并、Island body 独占、稳定合并顺序及 1/N worker 结果对照；可用平台补 ThreadSanitizer，不以普通压力测试替代数据竞争检查。
6. 正式性能测试统一使用 Release 和固定输入，输出带 CPU、操作系统、编译器/版本/选项、提交号及工作树是否有修改、场景参数、线程数、dt、预热帧数、采样帧数和重复次数的 JSON。各场景至少重复 5 次，报告分位数与运行间波动；时间按每固定步统计，创建销毁场景与稳态场景分开。性能断言不放普通单元测试，避免机器差异导致 CI 波动。
7. 先由基线为每个质量场景登记最大穿透、静止速度、漂移、睡眠耗时和能量漂移的容差及采样时长，再实现优化；容差修改必须单独记录原因，不能随性能结果临时放宽。Broadphase oracle 使用相同过滤规则检查无漏检，fat AABB 假阳性不当作正确性失败。

**Decisions**
- 保留角动量 L；网络同步角速度并不要求删除 L。当前网络尚无物理快照，本计划只冻结未来表示，不实现协议。
- “先更新角向再更新法向”定义为 Prepare 阶段先把角响应纳入法向/切向有效质量，迭代时以同一 delta lambda 同时更新 delta v 和 delta omega；不单纯交换写入顺序。
- 跨帧持久化 Pair、local anchors、法线、累计冲量及解释缓存所需的旧切线基/世界摩擦冲量、cachedDt、TOI 标记；r、世界响应、有效质量与 bias 每 substep 重建。缓存不是跨帧历史冲量总和。
- 第一版 local-anchor 匹配取代 feature ID；稳定 feature ID 是后续增强。
- 静态体不连接 Island；同一 Island 内暂不并行约束。
- 旧算法 A/B 路径仅为阶段迁移工具：聚焦测试、完整 Physics_Test、已冻结质量门槛与正式基准通过并保存差异后，在该阶段收尾删除旧实现和对应开关。后续阶段不要求持续维护 legacy 组合；旧 Broadphase 由暴力 oracle 接替，旧 Solver 由基线和不变量接替。单线程执行路径长期保留用于并发对照（Phase 6 落实为 workerCount=1 回退与 `SetParallelNarrowphaseEnabled` 开关），不保留重复算法实现。
- one-sided 首版保留方向语义，修正分母角缩放与实际响应不一致的问题；不承诺复制旧非物理 bounce boost 轨迹。首版仅使用位置修正消除穿透，不同时叠加 penetration velocity bias。
- 当前已有的静态/动态 body 与接触约束构成本轮 Island 范围；运动学 body、关节系统和 Island-local CCD 不借本次优化扩展实现。

**Agent ownership / merge order**
- Agent 0：基准统计与测试；Agent 1：PhysicsWorld 初始流水线；Agent 2：BroadPhase；Agent 3：Midphase/ContactManifold；Agent 4：CollisionDetector/Narrowphase 接线；Agent 5：RigidBody；Agent 6：ContactSolver；Agent 7：PhysicsWorld 最终集成；Agent 8：JobSystem；Agent 9：并行 Narrowphase；Agent 10：PhysicsIsland；Agent 11：并行 Solver；Agent 12：睡眠；Agent 13：CCD；Agent 14：最终验收。
- Agent 1 已完成初始 PhysicsWorld 流水线；Agent 0 保留场景矩阵、质量阈值和正式基线产物的补充任务。Agent 2（双 BVH）与 Agent 3/4（Midphase、ContactManifold、CollisionDetector 接线及 Phase 2 范围内的 PhysicsWorld 失效接线）已完成；补丁 3/4 已随 Phase 2 落地，补丁 8 的切向累计与固定步总冲量字段留 Phase 4。Agent 5（角响应）已完成，`ApplySolverAngularImpulse` 响应接口已冻结；下一批可并行启动 Agent 6（Prepare/Warm Start/累计冲量）与 Agent 8（JobSystem）。
- Agent 3 在 PairKey/查询接口冻结后可开发 Pair 池，实际 Broadphase 集成等待 Agent 2；Midphase/ContactManifold 契约及 Agent 5 的响应接口冻结后 Agent 4、Agent 6 可并行。随后 Agent 7 串行集成，在接触汇总与短暂事件回归通过后删除 MergeStepContacts 临时实现；Agent 8（JobSystem）与 Agent 9（并行 Narrowphase）已完成；Agent 10（Physics Island）已完成；Agent 11（并行 Island Solver）已完成；Agent 12（睡眠）可启动；随后 Agent 13（本轮仅全局 TOI 接线）；最后 Agent 14。Island-local CCD 另立计划。
- 高冲突文件必须单一所有者：ContactManifold.h 归 Agent 3，RigidBody.* 归 Agent 5，ContactSolver.* 归 Agent 6，PhysicsWorld.* 在初期归 Agent 1、最终集成归 Agent 7；后续 Agent 只通过冻结接口接入，避免并发修改。