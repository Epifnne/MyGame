# Physics 模块排障记录（physicsdebug）

## 适用范围

- 模块：Physics
- 场景：Demo 中球体落地后异常（原地打转、滚动异常、角速度失控）
- 目标：用真实运行数据定位根因，并形成可复用排障闭环

## 1. 初始现象

1. 球体落地后行为不符合预期：
- 有时看起来不弹起。
- 有时出现“原地打转”或不自然滑移。

2. 构建/运行侧现象：
- `cmake --build ... --target MyGame` 偶发 `Permission denied`。
- 根因是 `MyGame.exe` 被正在运行的进程占用，不是编译器错误。

## 2. 观测策略（先数据后改代码）

在 Demo 中增加球体遥测日志，按固定周期打印：

- 位姿与速度：`pos`、`vel`、`angVel`
- 接触状态：`groundContact`、`pair`
- 接触几何：`n`、`pen`
- 动力学判据：`relN`（法向相对速度）、`relT`（切向相对速度）
- 冲量：`nImpulse`
- 关键诊断量：`ballCpDist`（接触点到球心距离）

结论：这一步是本次排障成功的关键。

## 3. 关键实测数据与推理

### 3.1 判定逻辑误导

日志显示在接触期间经常出现：

- `groundContact=1`
- `relN > 0`

这会让求解器把当前接触误判为“分离”，法向冲量被跳过，导致行为不稳定（依赖位置修正撑住）。

### 3.2 真正致命点：接触点几何异常

最关键数据：

- `ballCpDist` 接触期出现 `2.4 -> 3.7`
- 球半径仅 `0.45`

这说明接触点远离球体表面，导致力臂过长，`r x J` 扭矩被异常放大，角速度快速失控，出现“打转/乱跑”视觉问题。

## 4. 尝试过但被回退的方案

1. 方案：当有微小穿透时，强制注入 penetration bias velocity。
2. 结果：会引入能量注入，角速度进一步发散。
3. 处理：已回退该策略，不保留到最终版本。

## 5. 最终修复方案

### 5.1 接近判定更稳健

在 `EnsureClosingVelocity` 中采用双通道判定：

1. 优先使用接触点相对速度判断接近/分离。
2. 若接触点受旋转影响出现“假分离”，回退到质心线速度判定。

目的：避免旋转接触时误判导致冲量被错误跳过。

### 5.2 约束球体接触力臂

在接触求解时对球体进行力臂约束：

- 若 A 是球：`ra = normal * radiusA`
- 若 B 是球：`rb = -normal * radiusB`

目的：即使接触点估计有偏差，也不允许产生远超球半径的扭矩臂。

### 5.3 窄相接触点细化（Sphere 参与时）

在 NarrowPhase 中对 Sphere 参与的接触点进行二次细化，减少 EPA 接触点偏离几何表面的风险。

## 6. 修改文件清单

- `Runtime/include/Physics/ContactSolver.h`
- `Runtime/src/Physics/ContactSolver.cpp`
- `Runtime/src/Physics/NarrowPhase.cpp`
- `Game/src/Core/GameApp.cpp`（添加遥测）
- `Docs/Architecture.md`（同步更新）

## 7. 验证结果

1. 单元测试：Physics 相关测试通过（8/8）。
2. Demo 运行：
- 角速度不再爆炸式增长。
- 行为从“异常打转”回到可控滚动/弹跳。
- `relT` 与 `angVel` 曲线明显收敛。

## 8. 复现与验证命令（Windows）

```powershell
Set-Location E:\MyGame
cmake --build Build -j 4 --target Physics_Test
E:\MyGame\MyGame\tests\Physics_Test.exe 2>&1
```

```powershell
taskkill /IM MyGame.exe /F 2>&1
Set-Location E:\MyGame
cmake --build Build -j 4 --target MyGame
Set-Location E:\MyGame\MyGame
.\MyGame.exe 2>&1
```

## 9. 给其他模块的复用模板

后续可以按同样方式创建：

- `Docs/graphicsdebug.md`
- `Docs/resourcedebug.md`
- `Docs/networkdebug.md`

建议固定结构：

1. 现象
2. 观测数据
3. 假设
4. 验证
5. 回退记录
6. 最终修复
7. 回归结果
8. 可复制命令

这样每个模块都有独立排障资产，互不干扰，也方便团队检索。