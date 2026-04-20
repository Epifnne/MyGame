# MyGame 引擎架构审查与优化建议

> 基于 2026-04-16 代码库快照的全面审查。
> 按优先级分级：P0（必须修复）→ P3（长期考虑）。

---

## 总体评价

当前引擎已具备可运行的 Core / ECS / Graphics / Physics / Platform 五个模块，分层架构清晰（ThirdParty → Shared → Runtime → Game），模块间依赖方向正确。以下问题按严重程度排序，旨在帮助引擎从"能跑"走向"可维护、可扩展、可测试"。

---

## P0 — 必须修复

### 1. 所有权语义不一致

**现状**：所有权管理在不同模块中混用裸指针、`unique_ptr`、`shared_ptr`，且无统一规范。

| 位置 | 指针类型 | 问题 |
|------|---------|------|
| `Engine::m_window` / `m_renderer` | 裸指针 `Window*` / `Renderer*` | 拥有但不管理，依赖手动 `CleanupSubsystems()` |
| `Renderer::Submit()` | 裸指针 `Mesh*` / `Material*` | 非拥有，但调用方需保证在 `Flush()` 前有效 |
| `ShaderManager` | `shared_ptr<Shader>` | 合理但与其他模块的 `unique_ptr` 策略不统一 |
| `Physics::CollisionShape` | `shared_ptr` | 合理 |

**建议**：
- `Engine` 持有 `Window` / `Renderer` 应改为 `std::unique_ptr`，利用 RAII 自动管理。
- `Renderer::Submit()` 的 `Mesh*` / `Material*` 若为非拥有引用，改为 `const Mesh&` / `const Material&` 或文档化非拥有约定。
- 制定项目级规范：`unique_ptr` 表独占所有权、`shared_ptr` 仅在真正共享时使用、裸指针仅在明确非拥有场景下使用。

**涉及文件**：
- `Runtime/include/Core/Engine.h`
- `Runtime/include/Graphics/Renderer.h`

---

### 2. Registry::GetComponent const 版本存在未定义行为

**现状**：
```cpp
template<typename T>
const T& GetComponent(Entity e) const {
    auto it = m_pools.find(std::type_index(typeid(T)));
    // 若 it == m_pools.end()，下一行解引用空迭代器 → UB
    return static_cast<const ComponentPoolWrapper<T>*>(it->second.get())->Get(e);
}
```

**建议**：添加 `assert` 或抛出明确异常：
```cpp
if (it == m_pools.end())
    throw std::runtime_error("Component type not registered");
```

**涉及文件**：
- `Runtime/include/ECS/Registry.h` — `GetComponent(Entity e) const`

---

## P1 — 重要改进

### 3. ECS 组件存储效率

**现状**：`ComponentPool<T>` 使用 `std::vector<std::optional<T>>`，以 Entity ID 直接作为下标寻址。

**问题**：
- 实体 ID 为 1000 时，即使只有 10 个实体拥有此组件，也要分配 1000 个 `optional<T>` 的空间。
- `Erase()` 中 `std::remove` 对 `m_entities` 列表操作为 $O(n)$。
- `resize(e + 20)` 增量策略可能导致频繁 resize。

**建议**：迁移到 **Sparse Set** 存储：
- 紧凑的 `dense` 数组保证缓存友好的遍历。
- `sparse` 数组提供 $O(1)$ 查找。
- 删除操作为 $O(1)$（swap-and-pop）。
- 已有注释提到此方案，建议优先实施。

**涉及文件**：
- `Runtime/include/ECS/ComponentPool.h`

---

### 4. 固定时间步重复实现

**现状**：`GameLoop` 和 `PhysicsWorld::Step()` 各自独立维护固定步长累加器。

**问题**：
- 两套累加器逻辑重复，且步长参数可能不一致。
- `GameLoop` 的固定步长回调已经调用 `Engine::FixedUpdate()`，`PhysicsWorld::Step()` 内部又做一次固定步长拆分 → 实际上物理会被**双重累加**。

**建议**：
- 统一由 `GameLoop` 的固定步长驱动物理更新。
- `PhysicsWorld` 提供 `FixedStep(float dt)` 单步接口（已有），移除 `Step()` 中的累加器逻辑，由外部驱动。

**涉及文件**：
- `Runtime/include/Core/GameLoop.h`
- `Runtime/include/Physics/PhysicsWorld.h` — `Step()` 和 `m_accumulator`

---

### 5. Physics-ECS 集成断裂

**现状**：`PhysicsWorld` 维护独立的 `unordered_map<uint32_t, RigidBody>`，与 ECS 的 `Registry` 完全独立。`GameApp.cpp` 中手动将物理位置写回 ECS 组件。

**问题**：
- 手动同步容易遗漏，新增实体/组件时需在两处维护。
- `PhysicsSystem` 虽然存在，但实际未被 `SystemManager` 管理。
- 物理 ID 与 Entity ID 不统一，需额外映射。

**建议**：
- 让 `PhysicsSystem::Update()` 自动完成 ECS → PhysicsWorld → ECS 的双向同步。
- 统一使用 `Entity` ID 作为物理体标识，或在 `RigidBody` 中持有 `Entity` 引用。
- 将 `PhysicsSystem` 注册到 `ECS::World` 的 `SystemManager` 中，由 ECS 驱动调度。

**涉及文件**：
- `Runtime/include/Physics/PhysicsSystem.h`
- `Runtime/include/Physics/PhysicsWorld.h`
- `Game/src/Core/GameApp.cpp`

---

### 6. EventBus 缺陷

**现状**：
```cpp
// 订阅 — 无返回句柄
void Subscribe(std::function<void(const Event&)> fn);
// 无 Unsubscribe 方法
```

**问题**：
- **无法取消订阅**：System 销毁后其 listener 仍存在于 EventBus 中 → 悬挂引用。
- **迭代器失效**：`Emit()` 遍历 listener 时若触发 `Subscribe()` → `m_listeners` 的 vector 可能 realloc → 迭代器失效 → 未定义行为。
- **异常不安全**：某个 listener 抛异常 → 后续 listener 不会被调用。

**建议**：
- `Subscribe()` 返回句柄（如 `uint64_t` 或 `Connection` 对象），支持 `Unsubscribe(handle)`。
- `Emit()` 时拷贝 listener 列表或使用延迟添加/删除队列。
- 考虑 `try-catch` 包裹每个 listener 调用或记录错误后继续分发。

**涉及文件**：
- `Runtime/include/ECS/EventBus.h`

---

### 7. Component.h 混入具体类型

**现状**：`Runtime/include/ECS/Component.h` 定义了 `Transform2D`、`Velocity2D`、`Transform3D`、`Rotation` 等具体组件。

**问题**：
- 这些是**游戏特定组件**，不应属于引擎 Runtime 层。
- Runtime 层对 Game 层的组件类型产生了反向依赖意识。
- 新增游戏组件不应修改引擎代码。

**建议**：
- 将具体组件类型移至 `Game/include/Components/`（部分已是 `using` 别名）。
- `Runtime/ECS/Component.h` 仅保留组件基础设施（如类型标识、注册宏等），或直接删除此文件。

**涉及文件**：
- `Runtime/include/ECS/Component.h`
- `Game/include/Components/` — 已有 using 别名

---

## P2 — 建议改进

### 8. 无日志/错误报告系统

**现状**：所有模块在失败时静默返回 `false` 或 `nullptr`，无任何日志输出。

**影响**：运行时问题（shader 编译失败、资源加载错误、物理异常值）无法诊断。

**建议**：
- 引入轻量日志系统（可基于 `spdlog` 或自研简单版本）。
- 定义日志级别：`Trace`/`Debug`/`Info`/`Warn`/`Error`/`Fatal`。
- 在关键路径添加日志：子系统初始化、资源加载、错误分支。
- 放置于 `Runtime/include/Core/Log.h`。

---

### 9. 无场景管理

**现状**：所有实体和逻辑在 `GameApp.cpp` 中平铺创建，无场景概念。

**影响**：无法实现关卡切换、场景序列化/反序列化、编辑器场景保存。

**建议**：
- 引入 `Scene` 类，持有 `ECS::World` + `PhysicsWorld` + 场景元数据。
- `SceneManager` 管理场景栈（加载/卸载/切换）。
- 场景序列化可结合 `nlohmann_json`（已下载未集成）。

---

### 10. 碰撞体 1:1 限制

**现状**：`PhysicsWorld` 使用 `unordered_map<uint32_t, Collider>` 存储碰撞体，键为 `bodyId`。

**影响**：每个刚体最多只能有一个碰撞体。真实引擎需要复合碰撞体（如角色：胶囊体 + 盒子 + 球体组合）。

**建议**：改为 `unordered_map<uint32_t, std::vector<Collider>>` 或引入 `CompoundShape`。

**涉及文件**：
- `Runtime/include/Physics/PhysicsWorld.h`

---

### 11. 无图形后端抽象 (RHI)

**现状**：`Shader`/`Mesh`/`Texture` 直接调用 OpenGL API（`glCreateProgram`、`glGenBuffers` 等），与 OpenGL 强耦合。

**影响**：未来无法支持 Vulkan、DirectX、Metal 等后端。

**建议**：
- 当前阶段不需要立即实施，但应规划 RHI（Render Hardware Interface）抽象层的引入时机。
- 隔离 GPU 资源创建/绑定/销毁到后端接口类中（如 `RHIDevice`、`RHICommandList`）。
- 现有 OpenGL 实现作为 `OpenGLDevice` 后端。

---

### 12. Resource 模块完全缺失

**现状**：14 个头文件全部为空。`GameApp.cpp` 中直接用硬编码路径加载 shader 和纹理。

**影响**：
- 无法统一管理资源生命周期。
- 无法实现热重载。
- 路径硬编码导致跨平台/打包后失效。

**建议**：分阶段推进，首先实现最小可用集：
1. **Phase 1**：`FileSystem`（虚拟路径映射）+ `ResourceManager`（统一 load/get 接口）。
2. **Phase 2**：`ResourceCache`（引用计数 + LRU 淘汰）+ 具体 Loader（Texture/Shader/Mesh）。
3. **Phase 3**：`AssetDatabase` + `HotReloadWatcher`。

---

## P3 — 长期考虑

### 13. GLOB_RECURSE 构建风险

**现状**：`Runtime/CMakeLists.txt` 使用 `file(GLOB_RECURSE ...)` 收集源文件。

**问题**：新增/删除 `.cpp` 文件后，CMake 不会自动重新配置，导致构建不完整或残留。

**建议**：显式列出源文件，或在 `CMakeLists.txt` 中添加 `CONFIGURE_DEPENDS`（CMake 3.12+）作为过渡方案。

---

### 14. 零线程安全保障

**现状**：
- `Engine::m_isRunning` 可通过 `Exit()` 从任意线程设置，无原子/锁保护。
- `Registry`、`EventBus` 等全局共享状态无同步机制。

**当前影响**：单线程引擎暂无问题。但未来引入多线程渲染、异步资源加载、任务系统时将成为阻碍。

**建议**：
- `m_isRunning` 改为 `std::atomic<bool>`。
- 规划时记录哪些子系统需要线程安全，提前为关键接口预留锁/无锁策略。

---

### 15. 缺少 Debug/Profiling 基础设施

**建议**：
- 帧计时器/性能统计（部分已有 `Time::GetFPS()`）。
- 物理调试绘制（线框碰撞体、接触点可视化）。
- 内存分配追踪。
- ImGui 集成用于运行时调试面板（已下载未集成）。

---

### 16. 三个已下载未集成的第三方库

| 库 | 状态 | 建议用途 |
|---|------|---------|
| **assimp** | 目录存在，CMake 未引用 | `MeshLoader` 的后端 |
| **imgui** | 目录存在，CMake 未引用 | Debug UI / 编辑器 |
| **nlohmann_json** | 目录存在，CMake 未引用 | 场景序列化、配置文件 |

---

## 关于单例模式的说明

`Engine`、`Input`、`Time` 使用 Meyer's Singleton。对于本项目（单一游戏引擎，不需要多引擎实例共存），这是合理且简洁的选择。C++11 保证局部 static 初始化线程安全，当前设计没有问题。

唯一需要注意的是**测试隔离**：单例状态可能跨测试泄漏。如果将来测试遇到此问题，可为单例添加 `Reset()` 方法供测试 fixture 调用，而不必重构为依赖注入。

---

## 推荐实施路线

```
Phase 1 (基础修复):
  P0-2 Registry UB 修复
  P0-1 Engine 裸指针 → unique_ptr
  P1-7 Component.h 具体类型下沉到 Game 层
  P2-8 引入日志系统

Phase 2 (ECS & Physics 强化):
  P1-3 ComponentPool → Sparse Set
  P1-6 EventBus 支持取消订阅 + 异常安全
  P1-5 PhysicsSystem 自动化 ECS 同步
  P1-4 固定步长统一

Phase 3 (资源 & 场景):
  P2-12 Resource 模块最小可用实现
  P2-9 Scene 管理
  P2-10 复合碰撞体

Phase 4 (长期):
  P2-11 RHI 抽象层
  P3-14 线程安全基础
  P3-15 Debug/Profiling
```

---

## 附：当前模块实现状态速查

| 模块 | Headers | Source | 状态 |
|------|---------|--------|------|
| Core | 4 | 4 | ✅ 已实现 |
| ECS | 9 | 0 (header-only) | ✅ 已实现 |
| Graphics | 11 | 10 | ✅ 已实现 |
| Physics | 14 | 0 (header-only) | ✅ 已实现 |
| Platform | 2 | 1 | ✅ 已实现 |
| Resource | 14 | 0 | ❌ 全部空文件 |
| Tools | — | — | ❌ 未开发 |

| 测试 | 用例数 | 覆盖范围 |
|------|--------|---------|
| ECS_Test | 2 | Entity CRUD、组件访问 |
| Graphics_Test | 2 | Camera、Material 数据结构 |
| Physics_Test | 3 | 重力积分、静态体、碰撞接触 |
| **总计** | **7** | 覆盖偏低，EventBus/Raycast/多 Pass 等未覆盖 |
