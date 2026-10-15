# Improvement Plan

本文档记录代码核对（2026-09-25）中发现的已知缺口及对应的优化方向，按模块组织。每条标注：现状、优化方向、预期收益与优先级建议。

2026-10-09 实施状态：已完成核心范围 1.1、2.1、2.2、2.3；2.3 按项目要求采用 archetype 而非原定 sparse-set。1.2、2.4、2.5、2.6 与可选的时间百分位统计仍保留为后续工作。

## 1. Core

### 1.1 Time 服务统一接入主循环（已完成）

**现状**

- [Time.h](../Runtime/include/Core/Time.h) 统一维护 steady_clock 采样、截断后的模拟帧间隔/累计时间、墙钟累计时间/FPS、帧数及固定步长累加器。
- [Engine::MainLoop](../Runtime/include/Core/Engine.h) 每帧工作开始前更新 Time，随后由 GameLoop::TickCurrentFrame 消费；独立 GameLoop::Tick 则自行更新所用 Time。避免两个时钟及重复采样。
- Start 重置计时，Resume 重新设置采样点以排除暂停时长；固定步长和最大帧间隔校验为有限正值。默认步长、截断值及 update → fixedUpdate → render 顺序保持不变。

**优化方向**

1. 已完成：固定累加器下沉到 Time，提供 GetDeltaTime / GetFixedDeltaTime / GetAccumulatedTime / ConsumeFixedStep。
2. 已完成：Engine 与回调共享同一次帧采样，FPS 使用未截断的墙钟时间，避免低帧率统计失真。
3. 未实施的可选项：帧时间直方图/百分位统计（p50/p95/p99），与 benchmark 体系打通。

**收益**：时间源唯一化；GameLoop 瘦身；物理与动画等系统可各自按固定/可变步长消费同一时钟。

**优先级**：中。功能不受影响，但属于架构正确性问题，建议尽早做，改动面小（Engine/GameLoop/Time 三个文件）。

### 1.2 GameLoop 缺少关闭阶段

**现状**：`GameLoop` 只有 Start/Stop/Pause/Resume，关闭流程由 `main()` 中 `game->Shutdown()` + `engine.Shutdown()` 组合完成（[main.cpp](../Game/src/main.cpp#L61-L64)）。

**优化方向**：为 GameLoop 增加显式的 shutdown 回调挂点（与 init/update/render 同级），引擎生命周期收口到 GameLoop 编排内。

**优先级**：低。当前流程可用，仅影响生命周期管理的对称性。

## 2. ECS

**总体现状**：全部头文件内联实现，`Runtime/src/ECS` 为空。已具备代际实体句柄、archetype 列式存储、多组件视图；系统仍按注册顺序驱动，EventBus 仍同步分发。

### 2.1 实体代际校验（已完成）

**现状**：[Entity.h](../Runtime/include/ECS/Entity.h) 使用 uint64_t：低 32 位 index + 高 32 位 generation；EntityPool::IsAlive 同时校验范围、存活位与代际。销毁/复用后旧句柄失效，代际耗尽的槽位永久退役。

**优化方向**

1. 已完成：句柄打包、EntityIndex / EntityGeneration / MakeEntity 辅助函数，NullEntity 保持为 0。
2. 已完成：Registry/World 暴露 IsAlive，所有组件访问/修改验证句柄；无效访问抛出 invalid_argument，不再静默写入。
3. 当前调用方使用完整 Entity 类型及标准整数哈希，无实体序列化路径。未来新增持久化/网络编码须使用完整 64 位句柄，不能截断为旧 uint32_t。

**收益**：消除悬空实体引用的静默错误；这是 ECS 正确性的底线，其他优化都建立在此之上。

**优先级**：高。代价小、收益大，且越晚改波及面越大（序列化、网络、UI 事件都会持有 Entity）。

### 2.2 多组件查询与视图（已完成）

**现状**：[Registry.h](../Runtime/include/ECS/Registry.h) 提供 View<Ts...>().Each，按 archetype 匹配组件交集，直接遍历组件列；const 视图传递 const 引用。EntitiesWith<Ts...> 保留为无序句柄快照。游戏移动/旋转及 MetalCube 更新/渲染已迁移。

**优化方向**

1. 已完成：改用 archetype 级签名匹配，内层无逐实体存在性检查/定位查找，无快照分配。
2. 已完成：View::Each 期间禁止结构修改并抛出 logic_error；如需增删，先收集句柄，迭代后再执行。嵌套视图和回调异常均有迭代保护。

**收益**：系统代码去样板化；为阶段调度（2.4）提供查询基础。

**优先级**：中高。每写一个多组件系统都在为这个缺口付利息。

### 2.3 archetype 紧凑存储（已完成，替代 sparse-set 方案）

**现状**：[Archetype.h](../Runtime/include/ECS/Archetype.h) 按有序组件类型签名分组，实体数组与每种组件的 vector<T> 按行对齐，不再使用按实体 ID 索引的 optional 池。

**实施内容**：组件增删迁移到目标 archetype，保留共同组件；源行 swap-and-pop 并修复被交换实体的 row。迁移前完成组件构造与目标列容量准备，要求组件 noexcept 移动构造/赋值，支持 move-only 和过对齐类型。组件引用可能因结构修改失效；空 archetype 与列容量保留复用。

**收益**：组件列连续，去除 ID 稀疏空洞；行删除与该 archetype 的组件种数成正比，不再扫描实体列表。性能对照见 [Benchmark README](../Benchmark/README.md)：相同 10 万实体/200 轮更新、Release-O3 三次运行中位数，旧查询热路径 2052.45ms，新 View 热路径 8.0493ms。此结果包含存储与查询 API 共同收益，不代表整帧或结构迁移性能。

**优先级**：中。性能向优化，建议在 2.1/2.2 之后做（接口稳定后换存储实现）。

### 2.4 系统阶段分组调度缺失

**现状**：`SystemManager` 按注册顺序逐个 `Update`（[SystemPool.h](../Runtime/include/ECS/SystemPool.h#L11-L15)），无输入/模拟/渲染前后阶段划分。

**优化方向**

1. `System` 增加 `Stage` 枚举属性（Input / Simulation / PreRender / PostRender），SystemManager 按阶段分组驱动。
2. 预留组内并行调度接口（结合现有 JobSystem），阶段间保留显式屏障。

**收益**：帧内执行顺序显式化，消除"注册顺序即正确性"的隐式依赖；为系统级并行铺路。

**优先级**：中。

### 2.5 EventBus 仅同步分发

**现状**：`Emit` 在调用栈上直接执行全部回调（[EventBus.h](../Runtime/include/ECS/EventBus.h#L16-L23)），无队列、无退订。`System::Update` 拿不到 EventBus，事件只能经 World 门面旁路传递。

**优化方向**

1. 增加 `Queue<Event>(...)` 延迟分发：帧内先入队，在帧级调度点统一 `Dispatch()`，避免系统更新中途触发级联修改。
2. 增加订阅句柄与 `Unsubscribe`，支持监听器生命周期管理。
3. `System::Update` 签名扩展为接收 World/上下文，打通系统 → 事件的直接通路。

**收益**：消除"emit 重入导致迭代器失效/半更新状态"一类隐患；事件流成为可推理的帧级数据流。

**优先级**：中。当前同步模型在系统数量少时无害，系统间事件变密后风险上升。

### 2.6 实现下沉到 src（编译期优化）

**现状**：ECS 全模板/内联在头文件，`src/ECS` 为空目录。每处 include 都重复实例化，编译时间随使用点线性膨胀。

**优化方向**：非模板部分（EntityPool、EventBus 的类型擦除底座、SystemManager）下沉为 .cpp 实现；模板组件池保留头文件但配合显式实例化常用组件类型。

**优先级**：低。纯编译期收益，功能无关。

## 3. 关联建议（实施顺序）

```text
2.1 代际校验 → 2.2 多组件视图 + 2.3 archetype 存储（已完成）
1.1 Time 收口（已完成）
2.4 阶段调度 / 2.5 事件队列（系统变多后按需）
2.6 编译期下沉（项目变大后按需）
```

## 4. 测试要求

- 已覆盖 2.1/2.2/2.3：[RuntimeECS_SmokeTest.cpp](../Tests/ECS/RuntimeECS_SmokeTest.cpp) 验证旧句柄、组件迁移/swap 行修复、move-only/对齐、查询交集/空查询/const、结构修改防护、构造异常，以及 5000 次随机操作与参考模型对照。
- 已覆盖 1.1：[Time_Test.cpp](../Tests/Common/Time_Test.cpp) 验证截断与墙钟 FPS、固定步长余量、同源回调/无重复采样、暂停/恢复/重启及非法参数。
- [ECSBenchmark.cpp](../Benchmark/ECSBenchmark.cpp) 提供带校验和的 JSON 查询微基准；前后对照记录见 Benchmark README。
- 2.5 后续实施时补充：事件队列的"同帧入队不立即触发回调"与重入防护用例。
