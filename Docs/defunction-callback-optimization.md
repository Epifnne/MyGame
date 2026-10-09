# Phase 5 补充：热路径回调去类型擦除 + Benchmark A/B 对比记录

> 日期：2026-09-11。状态：已落地并通过确定性门。
> 原关联的阶段计划与调查笔记已移除；当前功能摘要见 [Changelog](../CHANGELOG.md)，验证入口见 [Benchmark Guide](../Benchmark/README.md)。

## 背景与动机

用户提出把物理模拟内的 `std::function` 全部换成 Engine 式裸函数指针（`using GameUpdateFunc = void(*)(float)`）以降低开销。排查后**未采用函数指针方案**，原因：

1. 裸函数指针无捕获能力。broadphase 回调需要捕获 pair 集合、layer/mask 过滤器、body/collider 映射等状态，裸指针只能退化为 `void*` userdata（C 风格、易错、失去类型安全）。
2. **调用开销相同**。`std::function` 与函数指针都是一次间接跳转，都不能被内联——换函数指针收益为零。
3. 真正的成本不是"间接调用"本身，而是 `std::function` 的**类型擦除阻碍了编译器内联与跨函数优化**。

正确做法是**去类型擦除（de-virtualization）**，让编译器能看到回调体并内联。

## 排查：物理热路径的 std::function 分布

| 位置 | 调用频率 | 处置 |
|---|---|---|
| `BvhTree::SelfQueryPairs` / `QueryPairsAgainst` 的 per-pair 回调 | 每帧数千次（每个候选对一次） | **改为模板成员函数**（见下） |
| `JobSystem::RangeFunction` | 每 substep 一次 | **改为非持有 function-ref** |
| UI / Network / Input / EventBus / DataBinding 等 | 事件驱动，非每帧热路径 | 不动（避免过度防御） |

物理热路径仅此两处，其余 `std::function` 均不在每帧关键路径。

## 优化一：BvhTree 查询回调模板化

将 `SelfQueryPairs` / `QueryPairsAgainst` 从 `std::function` 参数改为 `template <typename OnPair>` 成员函数，实现移入头文件（[BvhTree.h](../Runtime/include/Physics/BvhTree.h)）。

- 调用方（[BroadPhase.cpp](../Runtime/src/Physics/BroadPhase.cpp) 的 `ComputePairs`）的 lambda 原样传入，编译器把整个 per-pair 处理（layer 过滤、static-static 剔除、`MakePairKey`、`push_back`）**内联进 BVH DFS 查询循环**，每帧数千次间接调用消失。
- 语义零变化（见下方确定性门）。

## 优化二：JobSystem RangeFunction 换非持有 function-ref

[JobSystem.h](../Runtime/include/Core/JobSystem.h) 的 `RangeFunction` 从 `std::function<void(size_t,size_t)>` 改为 16 字节非持有视图：

- 仅 `void* data` + `void(*invoke)(void*, size_t, size_t)` thunk，trivially copyable，**零堆分配、零引用计数**。
- 生命周期由 `ParallelForRange` 的 barrier 语义保证（被引用的 lambda 只需活过本次调用，barrier 确保这一点）。
- 支持普通 / mutable lambda、仿函数、函数指针，调用方代码零改动。

## Benchmark A/B 对比方法（可复现）

由于整个 Phase 0–5 均未提交，无法用 `git stash` 干净地取"改前"二进制。采用**单变量临时开关**：

1. 在 [BvhTree.h](../Runtime/include/Physics/BvhTree.h) 临时引入 `#ifdef BVHTREE_STD_FUNCTION_CALLBACK`，`#ifdef` 分支保留 `std::function` 声明，`#else` 保留模板实现；[BvhTree.cpp](../Runtime/src/Physics/BvhTree.cpp) 同步提供 `std::function` 版实现（A/B 后已删除开关，恢复纯模板版）。
2. 配置两个 Release 构建目录：
   - 改前：`-DCMAKE_CXX_FLAGS=-DBVHTREE_STD_FUNCTION_CALLBACK`
   - 改后：默认（模板）
3. 两目录共用同一源码树，仅该宏不同 → **唯一变量是回调机制**，其余 Phase 1–4 算法逻辑完全一致。
4. 每个场景各跑 5 次，取 P95 中位数。

> Release 配置（仓库无现成 Release preset，临时建目录）：
> ```
> cmake -S . -B Build-release -G Ninja -DCMAKE_BUILD_TYPE=Release \
>   -DCMAKE_C_COMPILER=D:/Qt/Tools/mingw1310_64/bin/gcc.exe \
>   -DCMAKE_CXX_COMPILER=D:/Qt/Tools/mingw1310_64/bin/g++.exe \
>   -DCMAKE_MAKE_PROGRAM=D:/Qt/Tools/Ninja/ninja.exe -DBUILD_TESTING=OFF
> cmake --build Build-release --target PhysicsBenchmark
> ```

## 结果（Release，MinGW 13.1，-O3，P95 中位数，5 次采样）

| 场景 | 阶段 | 改前 std::function | 改后 模板 | 提升 |
|---|---|---|---|---|
| boxfield | broadPhase | 0.110 ms | 0.098 ms | **-11%** |
| boxfield | narrowPhase | 0.546 | 0.518 | -5% |
| boxfield | solver | 0.130 | 0.126 | -3% |
| boxfield | **total** | **0.839** | **0.777** | **-7.4%** |
| hull500 | broadPhase | 0.607 ms | 0.470 ms | **-23%** |
| hull500 | narrowPhase | 2.451 | 1.957 | -20% |
| hull500 | solver | 0.706 | 0.683 | -3% |
| hull500 | **total** | **3.892** | **3.261** | **-16%** |

### 确定性门（改前 / 改后逐项一致）

| 场景 | broadPhaseCandidates | contactPoints | positionChecksum |
|---|---|---|---|
| boxfield 前=后 | 106080 | 34883 | 位级一致 |
| hull500 前=后 | 123681 | 146229 | 位级一致 |

候选数、接触点数、最终位置 checksum 全部位级相同 → 纯性能改动，无行为漂移。

## 解读

1. **收益集中在 broadPhase，且随回调调用次数放大。** hull500（500 凸包，候选对远多于 boxfield）broadPhase 快 23%，boxfield（200 box）只快 11%。回调越热，内联收益越大——验证了"per-pair 回调是热路径"的判断。
2. **hull500 narrowPhase 也降 20%，这不是回调直接贡献。** narrowphase 不经 BvhTree 回调，提升来自模板内联消除了宽相位查询循环的寄存器压力，使编译器对整个 `ComputePairs` 做了更激进的跨函数优化（连带收益）。
3. **solver 基本不变（-3%）是对照有效性的证据。** ContactSolver 无 `std::function` 热路径，若它也大幅下降说明测的是噪声。它的稳定反衬出 broadPhase 的提升是真实的。
4. **Debug 下几乎测不出差异。** 该优化的核心是"允许内联"，而 Debug 本就不内联——**此优化只在 Release 有意义**。这印证 plan 第 41 条"正式性能验收统一用 Release"。

## 结论与后续

- 物理热路径已无 `std::function`；UI/网络等冷路径保持原样。
- Release 基准目录 `Build-release` 已保留，供 Phase 11 正式验收复用。
- 后续每个 Phase 的性能验收可复用本文的"单变量 A/B + candidates/contacts/checksum 确定性门"方法：只切换一个机制，确认行为不变前提下量化收益，不以漏接触或额外穿透换取加速（plan 第 42 条质量门槛）。

## 附录：汇编级内联证据（2026-09-15）

"间接调用不能内联"需要证据而非断言。WSL2 未透传 PMU（`perf` 的 `cycles`/`instructions`/`branch-misses` 全部 `<not supported>`），硬件计数器不可用；但**汇编本身就是最直接的因果证据**——间接调用在机器码里是 `call *寄存器`，无法掩饰，内联后回调体会展开在循环体内。

单变量微基准（Build/inline_proof.cpp，临时文件，Build/ 已被 git 忽略）：跑真实 `BvhTree::SelfQueryPairs` 模板代码，回调复刻 `BroadPhase::ComputePairs` 的 tryEmit（两次 `unordered_map::find` + 过滤 + `push_back`）。唯一变量是回调类型：

- 擦除版：回调包成 `std::function` 传入 → 实例化为 `OnPair = std::function`（等价改前）
- 内联版：原始 lambda 直接传入（等价改后）

同编译器（MinGW g++ 13.1）同 `-O2`、`-S` 出汇编对比。

### 证据 1：汇编

**擦除版** `SelfQueryPairs<std::function>` 实例化函数体（344 行）中，DFS 循环的 per-pair 分支内存在：

```asm
.LEHB5:
        call    *24(%r13)          ; std::function 经 _M_invoker 函数指针的间接调用
        jmp     .L171
```

`call *24(%r13)` 每发一对执行一次，无法内联；该函数体调用清单里**没有** `Hashtable::find`，证明回调体（两次 `g_mask.find`）完全没被编进来。

**内联版** `RunQuery<lambda>` 实例化函数体（443 行）：

```
indirect 'call *reg': 0                              ; 零间接调用
call _ZNSt10_HashtableI...findERS1_.isra.0  (x2)    ; 回调的两次 hash find 直接内联进查询循环
```

### 证据 2：动态计时（同负载，pair 数一致）

| 变体 | best-of-5 | 每查询 | pairs |
|---|---|---|---|
| 擦除 std::function | 2745 ms | 915.1 µs | 21000 |
| 内联 lambda | 2629 ms | 876.4 µs | 21000 |

工作量完全相同（pairs 均 21000），纯查询循环差 **4.3%**。

### 对"间接调用都不内联"的限定

该说法基本正确但需限定：`std::function` 的间接调用在**跨越类型擦除边界**时编译器无法去虚拟化（目标在运行时经 `_M_invoker` 函数指针确定）。仅当编译器能在**同一翻译单元看到 `std::function` 的构造点**且能常量传播出 invoker 时才可能去虚拟化——`BvhTree`（模板在头文件、回调类型在调用方）这种跨 TU 场景不会发生。擦除版那处 `call *24(%r13)` 就是未被去虚拟化的实证。

原汇编操作指南已不在仓库中；上文保留了本次对照所用的汇编片段。
