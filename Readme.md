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
- `Engine` as the top-level entry: subsystem assembly, lifecycle management.
- `GameLoop` orchestrator: initialization, per-frame update, render drive, shutdown.
- Fixed-timestep accumulator plus variable-timestep update stages.
- Unified input service (keyboard/mouse state sampling, per-frame event caching).
- Time service: delta time, fixed-step accumulator, runtime statistics.

### ECS
- Entity handles with stable IDs and generational validation, pooled allocation and reuse.
- Type-erased component pools with contiguous storage for fast iteration.
- Query-driven system views; systems grouped and scheduled by stage (input, simulation, pre/post render).
- Publish/subscribe `EventBus` for decoupled cross-system communication.
- `World` facade aggregating registry, system pool, and event bus with frame-level scheduling.

### Graphics
- Forward rendering pipeline abstraction (`RenderPipeline`) with default and minimal (`SimplePipeline`) implementations.
- Camera, mesh, material, texture, and shader abstractions; shader/mesh/texture managers with caching and reuse.
- Unified render state description (depth, blend, rasterizer).
- **Shadow mapping**: directional light ShadowMap and Cascaded Shadow Maps (CSM) with cascade splitting, texel snapping stabilization, depth-only shadow passes, PCF filtering, and cascade boundary blending.

### Physics
- Deterministic fixed-step simulation pipeline: input sync → integration → broad phase → narrow phase → constraint/impulse solving → writeback to ECS.
- **Broad phase**: dynamic BVH (fat AABBs, incremental insert/remove, reinsert-on-move).
- **Narrow phase**: GJK + EPA contact generation with cached contact manifolds.
- Rigid body dynamics: mass, inertia tensor, quaternion orientation, force/torque accumulation.
- Contact solver: normal/friction impulses, position correction, one-sided contacts.
- **Continuous collision detection**: TOI search with sub-stepping to prevent tunneling.
- Physical materials (static/dynamic friction, restitution, combine rules), triggers, collision filtering.
- Spatial queries: raycast, shape sweep, overlap tests.
- Constraint abstraction for joints, distance, springs, and limits.

### Network
- Client-server architecture over TCP (reliable messages) and UDP (real-time state).
- Dedicated network thread with a thread-safe event queue, bridged to ECS via `EventBus`.
- Binary message serialization with custom type registration and ID-based dispatch.
- Packet buffering: fragmentation, reassembly, ordering, flow control.
- Sessions with authentication state, heartbeat, timeout detection, and one-shot reconnect.
- Server-authoritative state snapshots (delta compression) and client-side prediction with rollback (scaffolding in place, staged rollout).

### Resource
- Virtual file system abstraction over platform file IO.
- `ResourceManager` facade with load/get/release; state machine + reference counting per resource.
- Asset database mapping GUIDs to metadata; import pipeline converting source assets to runtime formats.
- Typed loaders for textures (mipmaps, color space), meshes (vertex layouts, submeshes, tangents), materials, and shaders (variant keys, reflection data).
- Hot-reload watcher: file change detection with incremental reload and dependency propagation.

### UI
- Two-layer UI: retained-mode widget tree for game UI + ImGui overlay for debugging/tooling.
- Screen Space and World Space canvases; anchor/absolute layout engine with margins and alignment.
- Hit-testing and event routing along the widget tree, bridged to the ECS `EventBus`.
- One-way data binding (observer pattern) from data sources to widgets.
- Style/theme system with runtime theme switching.
- Full widget set: Label, Button, Image, Panel, Slider, ListView, ScrollView, InputField, Toggle, ProgressBar, TreeView, TabView, DropDown, Dialog.

### Gameplay Framework
- Attribute sets with base values and stacked modifiers (Add / Multiply / Override).
- Gameplay effects: spec + runtime instance, modifiers, duration, granted tags.
- Gameplay tags: centralized registry with reference-counted gating.
- `GameplaySystem` facade aggregating attributes, tags, and effects per entity.

### AI
- Behavior trees: blackboard + node tree + frame-driven tick.
- Composite nodes (Sequence/Selector), decorators (Inverter/Repeat), and leaf nodes for conditions/actions.
- Type-safe blackboard for shared decision data; external services injectable via leaf node user data.

### Platform
- Window abstraction: creation/destruction, event polling, swap-chain presentation, size management.
- Platform utilities: paths, environment, clocks, system capability queries.

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
- [Physics Collision Optimization Plan](Docs/PhysicsCollisionOptimizationPlan.md)
- [Physics Debug Notes](Docs/physicsdebug.md)
- [CI Debug Playbook](Docs/CI_DEBUG_PLAYBOOK.md)
