# Unreal Chaos 物理架构（基于源码的笔记）

## 范围

本笔记基于对本地 Unreal 源码树的扫描：

- Engine/Source/Runtime/Experimental/Chaos
- Engine/Source/Runtime/Experimental/ChaosSolverEngine
- Engine/Source/Runtime/PhysicsCore
- Engine/Source/Runtime/Engine/Private/PhysicsEngine

代表性文件包括：

- Chaos 核心：`PBDRigidsSolver.h/.cpp`、`PhysicsSolver.h`、`Chaos/BoundingVolumeHierarchy.h`、`Chaos/CCDUtilities.cpp`
- 求解器引擎桥接：`ChaosSolverActor.h/.cpp`、`ChaosSolver.cpp`、`ChaosGameplayEventDispatcher.cpp`
- 运行时封装：`BodyInstance.cpp`、`BodySetup.cpp`、`CollisionQueryFilterCallback.cpp`
- 物理 API 层：`PhysicsCore.h`、`CollisionShape.h`、`PhysicsSQ.h`

## 高层次分层

Chaos 架构可分为四层：

1. PhysicsCore（面向引擎的 API 合同）
2. 引擎物理集成（BodyInstance、BodySetup、场景/组件桥接）
3. Chaos 求解器运行时（PBD 刚体求解器、广/窄相、约束、岛屿）
4. 求解器编排与游戏事件（ChaosSolverEngine）

## 模块职责

### 1）PhysicsCore

主要目的：为引擎/运行时系统提供稳定的物理类型系统和接口。

常见职责：

- 碰撞/查询类型和过滤回调接口
- 物理设置与初始化合同
- 上层求解器实现使用的形状与查询工具抽象

代表性文件：

- `PhysicsCore/Public/PhysicsCore.h`
- `PhysicsCore/Public/CollisionShape.h`
- `PhysicsCore/Public/PhysicsSQ.h`
- `PhysicsCore/Public/CollisionQueryFilterCallbackCore.h`

### 2）引擎物理集成

主要目的：将游戏对象/组件绑定到物理表示，并保持模拟状态同步。

常见职责：

- 刚体创建、归属、休眠/唤醒、运动/动力学切换
- 材质/碰撞配置映射到运行时查询/模拟标志
- 约束与查询桥接到世界场景

代表性文件：

- `Engine/Private/PhysicsEngine/BodyInstance.cpp`
- `Engine/Private/PhysicsEngine/BodySetup.cpp`
- `Engine/Private/PhysicsEngine/ConstraintInstance.cpp`
- `Engine/Private/PhysicsEngine/CollisionQueryFilterCallback.cpp`

### 3）Chaos 求解器核心

主要目的：以 PBD/XPBD 风格的迭代方式执行物理模拟。

常见职责：

- 粒子/刚体状态容器
- 广相加速结构（BVH/AABB 树）
- 窄相与流形生成
- 约束图/岛屿求解/迭代循环
- CCD 工具用于高速碰撞鲁棒性

代表性文件：

- `Experimental/Chaos/Public/PBDRigidsSolver.h`
- `Experimental/Chaos/Private/PBDRigidsSolver.cpp`
- `Experimental/Chaos/Public/Chaos/BoundingVolumeHierarchy.h`
- `Experimental/Chaos/Private/Chaos/CCDUtilities.cpp`
- `Experimental/Chaos/Private/Chaos/CCDModification.cpp`

### 4）ChaosSolverEngine（编排与事件）

主要目的：向游戏系统暴露求解器实例和事件管道。

常见职责：

- 求解器 Actor/组件生命周期
- 事件分发（碰撞/破碎/拖尾等游戏事件）
- 调试绘制与可视化调试器钩子

代表性文件：

- `Experimental/ChaosSolverEngine/Public/Chaos/ChaosSolverActor.h`
- `Experimental/ChaosSolverEngine/Private/Chaos/ChaosSolverActor.cpp`
- `Experimental/ChaosSolverEngine/Private/Chaos/ChaosGameplayEventDispatcher.cpp`

## 典型运行时数据流

1. 引擎层的组件/刚体设置创建物理代理。
2. 代理被插入到 Chaos 求解器场景结构中。
3. 每个固定步：
   - 收集外部力/速度
   - 生成广相候选对（BVH/AABB）
   - 生成窄相接触/流形
   - 迭代求解约束（包括摩擦/反弹）
   - 求解后写回变换/速度
4. 求解器事件被过滤/分发到游戏监听器。

## Chaos 设计模式对 MyGame 的启示

1. 保持 API 合同独立于求解器实现细节。
2. 将编排（世界步进/事件）与数学密集型求解器内核分离。
3. 保持 CCD 作为一级流水线阶段，而非后期补丁。
4. 优先采用具有明确阶段边界的迭代求解循环：
   - 积分
   - 检测
   - 求解
   - 后修正
5. 在求解器边界保留调试/事件钩子。

## 建议在 MyGame 中的映射

- 类 PhysicsCore：
  - 接口和 DTO（`RigidBody`、`Collider`、`ContactManifold`、材质/查询合同）
- 类引擎集成：
  - 游戏侧适配器，将组件/实体接入物理世界
- 类 Chaos 核心：
  - 模块化广相、窄相（GJK/EPA）、CCD 阶段、迭代求解器
- 类求解器引擎：
  - 世界级事件流与调试诊断层

## MyGame 的近期重构优先级

1. 将单面/平台策略从零散分支重构为显式接触策略模块。
2. 保持所有接触统一的法线/切线冲量求解，策略仅修改约束。
3. 增加求解器阶段遥测（前/后法线速度、穿透、冲量和）。
4. 在最终求解迭代前保留专用 CCD 阶段。
5. 增加如下场景测试：
   - 高自旋 + 地面反弹
   - 边缘/平台单面行为
   - 高速隧穿
