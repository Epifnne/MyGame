# MyGame Engine

A modular C++20 game engine with a reusable **Runtime** layer, an **ECS**-driven architecture, and a deterministic physics core. The engine targets Windows (MinGW/MSVC) and Linux (GCC) and is built with CMake + Ninja.

> For detailed architecture information, see [Docs/Architecture.md](Docs/Architecture.md).

## Architecture Overview

```text
MyGame/
├─ Runtime/        Reusable engine layer (stable API for the Game layer)
│  ├─ Core/        Engine entry point, game loop, input, time
│  ├─ ECS/         Entities, components, systems, event bus
│  ├─ Graphics/    Renderer, pipelines, materials, shadow mapping
│  ├─ Physics/     Deterministic stepping, collision, rigid body solver
│  ├─ Network/     Client-server networking, snapshots, prediction
│  ├─ Resource/    Asset import, loading, caching, hot reload
│  ├─ UI/          Retained-mode widget tree + ImGui debug overlay
│  ├─ Gameplay/    Attributes, gameplay effects, gameplay tags
│  ├─ AI/          Behavior trees with blackboard
│  ├─ Platform/    Windowing and OS abstraction
│  └─ Common/      Shared utilities (generic resource handles)
├─ Game/           Game-specific components, systems, and assets
├─ Shared/         Cross-boundary shared config and types
├─ Sample/         Runnable samples (e.g., physics collision demo)
├─ Benchmark/      Physics performance benchmarks
├─ Tests/          GoogleTest unit/integration tests wired into CTest
├─ Tools/          Local server demo and helper scripts
└─ ThirdParty/     assimp, glad, glfw, glm, googletest, imgui,
                   nlohmann_json, stb
```

Layering rules:

- `include/` holds interfaces and data definitions; `src/` holds implementations.
- `Game` depends on `Runtime` and `Shared` only — never directly on `ThirdParty`.
- Subsystems communicate through the ECS `EventBus` for loose coupling.

## Core Features

### Core & Game Loop
- `Engine` as the top-level entry: window/renderer assembly and lifecycle management.
- `GameLoop` orchestrator: fixed-timestep accumulator plus variable-timestep update stages, with render drive.
- Unified input service: keyboard/mouse state sampling with per-frame event caching, fed by GLFW callbacks.
- Time service: delta time, elapsed time, and FPS statistics (the fixed-step accumulator lives in `GameLoop`).

### ECS
- Header-only implementation under `Runtime/include/ECS`.
- Entity handles as stable numeric IDs with pooled allocation and free-list reuse (generation is tracked inside the pool; stale-handle validation is not enforced yet).
- Type-erased component pools keyed by `std::type_index`, storing components in per-entity-indexed vectors.
- Single-component queries; systems updated in registration order via the system manager.
- Synchronous publish/subscribe `EventBus` for cross-system communication.
- `World` facade aggregating registry, system manager, and event bus.

### Graphics
- OpenGL 3.3 Core backend (GLFW windowing + GLAD loader).
- Forward rendering pipeline abstraction (`RenderPipeline`) with default and minimal (`SimplePipeline`) implementations; draw submission and state application are handled by `Renderer::Flush`.
- Camera, mesh, material, texture, and shader abstractions; shader/mesh/texture managers with caching and reuse.
- Unified render state description (depth, blend, rasterizer, cull, color write mask) applied around draw calls.
- Shadow mapping (directional ShadowMap / CSM) is scaffolded at the interface level only: light and shadow-settings data structures exist, but the runtime shadow passes are not implemented yet.

### Physics
- Deterministic fixed-step simulation pipeline: integration → broad phase → midphase → narrow phase → island-based impulse solving, stepped via `PhysicsSystem` (ECS writeback is not wired up yet).
- **Broad phase**: dynamic BVH (fat AABBs, incremental insert/remove, refit with reinsert-on-move) with separate static/dynamic trees.
- **Midphase**: persistent pair pool with generational handles, cached contact manifolds with local-anchor matching and accumulated-impulse warm-start caches, Enter/Stay/Exit contact events.
- **Narrow phase**: GJK + EPA contact generation with up to 4-point manifolds, parallelized across worker threads via a shared `JobSystem`; independent physics islands are solved in parallel.
- Rigid body dynamics: mass, inertia tensor, quaternion orientation, force/torque accumulation, semi-implicit Euler integration.
- Contact solver: accumulated normal/friction impulses with friction-disc clamping, warm starting, position correction, one-sided contact normal redirection (static friction is stored but not yet used by the solver).
- **Continuous collision detection**: sampled TOI search with bisection refinement and sub-stepping to prevent tunneling.
- Physical materials (friction, restitution, combine rules), triggers, and layer/mask collision filtering.
- Spatial queries: AABB raycast; shape sweep and overlap queries are not implemented yet.
- Constraint abstraction exists as a minimal interface; concrete joint/distance/spring constraints are not implemented yet.

### Network
- Client-server messaging over TCP, with a platform socket wrapper that also supports UDP.
- Dedicated network thread with a thread-safe event queue; `NetworkSystem` bridges network events toward the ECS.
- Binary message serialization with type-ID-based dispatch.
- Packet buffering with fragmentation and reassembly.
- Client connection with heartbeat and one-shot reconnect; local server demo under `Tools/`; TCP loopback tests wired into CTest.
- Reliable-UDP channel, state snapshots, and client-side prediction/rollback are interface placeholders only (staged roadmap, not implemented).

### Resource
- Virtual file system abstraction: mount points, virtual path resolution, text/binary IO over the platform file system.
- `ResourceManager` facade with a sync/async loading pipeline (IO → decode → upload) and cache-based eviction via `shared_ptr` use counts.
- Asset database mapping GUIDs to metadata with virtual-path reverse lookup.
- Generic resource handles (`Runtime::Handle` in `Common/`) with typed aliases (`AssetHandle`, `MeshHandle`, `TextureHandle`).
- Typed loaders for textures, meshes, materials, and shaders currently wrap raw file data; format-specific parsing (mipmaps, vertex layouts, shader variants/reflection) is not implemented yet.
- Hot-reload watcher: modification-time polling and change detection; automatic reload and dependency propagation are not wired up yet.

### UI
- Header-only retained-mode widget tree prototype (the `src/UI` `.cpp` files are empty translation units).
- Canvas with Screen/World Space markers; anchor-based layout engine with margins.
- Hit-testing and event routing along the widget tree (bridging to the ECS `EventBus` is defined but not wired up yet).
- One-way data binding primitives: observable properties with subscribe/notify.
- Basic style sheet structure (global and per-type styles); runtime theme switching is not implemented.
- Widgets with basic interactions: Label, Button, Image, Panel, Slider, ListView, InputField, Toggle, ProgressBar, TreeView, TabView, DropDown; ScrollView and Dialog are minimal stubs.
- `DebugUI` is currently a text-line cache; ImGui integration is not implemented yet.

### Gameplay Framework
- Attribute sets with base values and stacked modifiers (Add / Multiply / Override).
- Gameplay effects: spec + runtime instance, modifiers, duration, granted tags.
- Gameplay tags: centralized name/ID registry; per-entity tag reference counting lives in `GameplaySystem`.
- `GameplaySystem` facade aggregating attributes, tags, and effects per entity.

### AI
- Behavior trees: blackboard + node tree + frame-driven tick.
- Composite nodes (Sequence/Selector), decorators (Inverter/Repeat), and leaf nodes for conditions/actions.
- Type-safe blackboard for shared decision data; external services injectable via leaf node user data.

### Platform
- Window abstraction over GLFW: creation/destruction, event polling, buffer swap, size and VSync management.
- Platform utilities (paths, environment, clocks, capability queries) are not implemented yet; `PlatformUtils` is a placeholder.

## Building (Windows)

The repo uses the Ninja generator with a fixed `Build` directory. The local preset uses Qt MinGW 13.1 with the CMake and Ninja shipped with Qt.

```powershell
Set-Location E:\MyGame
cmake --preset mingw-debug
cmake --build --preset mingw-debug
ctest --preset mingw-debug
```

Build or test a single target:

```powershell
cmake --build --preset mingw-debug --target Physics_Test
ctest --preset mingw-debug -R Physics
```

Do not switch generators inside an existing `Build` directory. If the old cache came from Visual Studio, NMake, or MinGW Makefiles, delete `Build/CMakeCache.txt` and `Build/CMakeFiles` before configuring with the preset.

CI and Release builds also use Ninja with platform-default compilers: GCC on Linux, MSVC on Windows.

## Testing & Benchmarks

- Unit and integration tests per module under `Tests/` (GoogleTest), wired into CTest.
- Physics performance benchmarks under `Benchmark/`, with JSON baselines for regression comparison.
- Runnable samples under `Sample/` and a local TCP server demo under `Tools/`.

## Documentation

- [Architecture (Chinese)](Docs/Architecture.md) — module responsibilities and layering contracts
- [Improvement Plan (Chinese)](Docs/ImprovementPlan.md) — known gaps and optimization directions from the 2026-09 code review
- [Physics Collision Optimization Plan](Docs/PhysicsCollisionOptimizationPlan.md)
- [Physics Debug Notes](Docs/physicsdebug.md)
- [CI Debug Playbook](Docs/CI_DEBUG_PLAYBOOK.md)
