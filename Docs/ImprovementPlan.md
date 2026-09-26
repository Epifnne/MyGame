# Improvement Plan

本文档记录代码核对（2026-09-25）中发现的已知缺口及对应的优化方向，按模块组织。每条标注：现状、优化方向、预期收益与优先级建议。

## 1. Core

### 1.1 Time 服务未接入主循环

**现状**

- `Time` 提供 deltaTime、累计运行时间与 FPS 统计（[Time.h](../Runtime/include/Core/Time.h)），但 `Engine::MainLoop` 从未调用 `Time::Get().Update()`，实例空转。
- 固定步长累加器位于 `GameLoop::Tick`（[GameLoop.h](../Runtime/include/Core/GameLoop.h#L40-L63)），而 Time 自身没有固定步长能力。两处时间逻辑割裂。

**优化方向**

1. 将固定步长累加器从 `GameLoop` 下沉到 `Time`：`Time::Update(now)` 内部维护 accumulator，对外提供 `DeltaTime()` / `FixedDeltaTime()` / `AccumulatedTime()`。GameLoop 只做调度，不再持有时间状态。
2. `Engine::MainLoop` 每帧首先调用 `Time::Get().Update()`，游戏逻辑与渲染统一从 Time 读取，消除多源时间。
3. 可选：Time 内增加帧时间直方图/百分位统计（p50/p95/p99），与 benchmark 体系打通。

**收益**：时间源唯一化；GameLoop 瘦身；物理与动画等系统可各自按固定/可变步长消费同一时钟。

**优先级**：中。功能不受影响，但属于架构正确性问题，建议尽早做，改动面小（Engine/GameLoop/Time 三个文件）。

### 1.2 GameLoop 缺少关闭阶段

**现状**：`GameLoop` 只有 Start/Stop/Pause/Resume，关闭流程由 `main()` 中 `game->Shutdown()` + `engine.Shutdown()` 组合完成（[main.cpp](../Game/src/main.cpp#L61-L64)）。

**优化方向**：为 GameLoop 增加显式的 shutdown 回调挂点（与 init/update/render 同级），引擎生命周期收口到 GameLoop 编排内。

**优先级**：低。当前流程可用，仅影响生命周期管理的对称性。

## 2. ECS

**总体现状**：全部头文件内联实现，`Runtime/src/ECS` 为空。功能处于"可用原型"水平：单组件查询、注册顺序驱动系统、同步 EventBus。

### 2.1 实体代际校验未生效

**现状**：`Entity` 是裸 `uint32_t`（[Entity.h](../Runtime/include/ECS/Entity.h#L9-L10)）；`EntityPool` 销毁时递增内部 generation 但 `IsAlive` 只查 ID 范围与存活位（[EntityPool.h](../Runtime/include/ECS/EntityPool.h#L35-L41)）。销毁并复用后，旧句柄数值相同，会再次判定为 alive——**这是真实的悬空句柄 bug 温床**。

**优化方向**

1. 将 generation 打包进 `Entity` 句柄本身（经典方案：`uint32_t = 20bit index + 12bit generation`，或升级到 `uint64_t` 换取更宽的代数位）。
2. `IsAlive` 同时比较 index 存活位与句柄 generation 是否等于池内当前 generation。
3. 配套更新 `NullEntity`、哈希与序列化路径。

**收益**：消除悬空实体引用的静默错误；这是 ECS 正确性的底线，其他优化都建立在此之上。

**优先级**：高。代价小、收益大，且越晚改波及面越大（序列化、网络、UI 事件都会持有 Entity）。

### 2.2 多组件查询与视图缺失

**现状**：Registry 仅支持 `EntitiesWith<T>()` 单组件查询（[Registry.h](../Runtime/include/ECS/Registry.h#L61-L65)）。系统若需"同时拥有 A 和 B 的实体"，只能查一个再手动过滤。

**优化方向**

1. 实现 `View<Ts...>`：选择池最小的组件类型作为驱动（size-driven iteration），其余池做存在性检查。
2. 若配合 2.3 的紧凑存储改造，可进一步做 SoA 连续迭代。

**收益**：系统代码去样板化；为阶段调度（2.4）提供查询基础。

**优先级**：中高。每写一个多组件系统都在为这个缺口付利息。

### 2.3 组件存储非紧凑

**现状**：组件存于 `std::vector<std::optional<T>>`，按实体 ID 索引（[ComponentPool.h](../Runtime/include/ECS/ComponentPool.h#L24-L30)）。实体 ID 稀疏时内存空洞大；`optional` 引入额外开销；`Entities()` 返回副本。

**优化方向**：改造为 sparse-set（dense array + sparse index），删除时 swap-and-pop，保持 dense 数组连续。文件底部注释（L48-53）已有该设计草案。

**收益**：迭代缓存友好、删除 O(1)、内存占用与活跃实体数成正比。

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
2.1 代际校验（正确性底线）
  └→ 2.2 多组件视图（接口定型）
       └→ 2.3 sparse-set 存储（接口稳定后的内部替换）
1.1 Time 收口（独立进行，随时可做）
2.4 阶段调度 / 2.5 事件队列（系统变多后按需）
2.6 编译期下沉（项目变大后按需）
```

## 4. 测试要求

- 2.1：新增"销毁-复用后旧句柄失效"用例（现有 [RuntimeECS_SmokeTest.cpp](../Tests/ECS/RuntimeECS_SmokeTest.cpp) 未覆盖）。
- 2.2：多组件查询的交集正确性与空池边界用例。
- 2.3：性能基准前后对照（建议纳入 Benchmark/ 现有 JSON 基线体系）。
- 2.5：事件队列的"同帧 emit 不立即触发回调"与重入防护用例。
