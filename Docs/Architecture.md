# MyGame Architecture

## 1. 架构目标

本文档只描述架构设计，不记录“已创建文件清单”或“当前仓库快照”。

核心目标：

1. Runtime 作为可复用引擎层，对 Game 提供稳定接口。
2. Physics 负责确定性步进、碰撞检测与刚体解算。
3. Resource 负责资产导入、加载、缓存、生命周期与热更新。
4. Network 负责客户端-服务器通信、消息序列化与状态同步。
5. UI 负责游戏界面渲染、控件管理与调试叠加层。

## 2. 项目预期目录结构

```text
MyGame/
├─ CMakeLists.txt
├─ Docs/
│  └─ Architecture.md
├─ Game/
│  ├─ CMakeLists.txt
│  ├─ include/
│  └─ src/
├─ Runtime/
│  ├─ CMakeLists.txt
│  ├─ include/
│  │  ├─ Core/
│  │  ├─ ECS/
│  │  ├─ Graphics/
│  │  ├─ Network/
│  │  ├─ Physics/
│  │  ├─ Platform/
│  │  ├─ Resource/
│  │  └─ UI/
│  └─ src/
│     ├─ Core/
│     ├─ ECS/
│     ├─ Graphics/
│     ├─ Network/
│     ├─ Physics/
│     ├─ Platform/
│     ├─ Resource/
│     └─ UI/
├─ Shared/
│  ├─ CMakeLists.txt
│  ├─ include/
│  └─ src/
├─ Tests/
│  └─ CMakeLists.txt
├─ ThirdParty/
└─ Tools/
```

说明：

1. include 只放接口与数据定义。
2. src 放实现与模块内部协作。
3. Game 依赖 Runtime 和 Shared，不直接依赖 ThirdParty。

## 3. Runtime/Core 头文件职责

### 3.1 模块分层

Core 采用以下主循环与基础服务流程：

1. 初始化引擎上下文与子系统。
2. 采集平台输入并更新输入状态。
3. 驱动固定步长与可变步长更新阶段。
4. 维护时间信息并计算帧间隔。
5. 调度游戏循环并处理退出条件。
6. 执行关闭流程与资源回收。

### 3.2 每个头文件（类）预期职责

- Engine.h：引擎总入口；负责子系统装配、生命周期管理与对上层启动接口。
- GameLoop.h：主循环编排器；组织初始化、逐帧更新、渲染驱动与退出流程。
- Input.h：输入服务接口；统一键鼠/手柄状态采样、查询与帧内事件缓存。
- Time.h：时间服务接口；提供 deltaTime、固定步长累加器与运行时间统计。

## 4. Runtime/ECS 头文件职责

### 4.1 模块分层

ECS 采用以下数据与调度流程：

1. 创建/回收实体并维护生命周期。
2. 注册组件类型并管理组件存储。
3. 按查询条件构建系统处理视图。
4. 驱动系统按阶段更新（输入、模拟、渲染前后）。
5. 分发事件并处理系统间解耦通信。
6. 维护世界级上下文并对外提供统一入口。

### 4.2 每个头文件（类）预期职责

- World.h：ECS 世界门面；聚合 Registry、SystemPool 与事件总线并提供帧级调度入口。
- Registry.h：注册中心；负责实体与组件关系管理、查询构建与批量结构变更。
- Entity.h：实体句柄抽象；提供稳定 ID、代际校验与轻量操作接口。
- EntityPool.h：实体池；负责实体 ID 分配、复用与代际递增策略。
- Component.h：组件基抽象与类型标识；定义组件类型元信息与通用访问约定。
- ComponentPool.h：组件池；按类型连续存储组件数据并提供高效增删改查。
- System.h：系统基接口；定义系统生命周期（初始化、更新、销毁）与执行阶段约束。
- SystemPool.h：系统容器；维护系统注册顺序、阶段分组与启停状态。
- EventBus.h：事件总线；提供发布/订阅机制、事件分发队列与跨系统解耦通信。

## 5. Runtime/Graphics 头文件职责

### 5.1 模块分层

Graphics 采用以下渲染流水线：

1. 收集场景可见对象与相机参数。
2. 组织渲染队列并进行批次/状态排序。
3. 绑定渲染管线、着色器与资源。
4. 执行绘制调用并提交到图形后端。
5. 管理帧内状态切换与后处理阶段。
6. 输出到目标缓冲并与窗口系统同步。

### 5.2 每个头文件（类）预期职责

- Renderer.h：渲染系统入口；驱动每帧渲染流程并协调管线、资源与提交顺序。
- RenderPipeline.h：渲染管线抽象；定义渲染阶段组织、Pass 编排与扩展点接口。
- DefaultPipeline.h：默认管线实现；提供通用前向渲染流程与基础渲染阶段配置。
- SimplePipeline.h：轻量管线实现；用于最小可运行渲染路径与调试场景。
- Camera.h：相机数据与变换接口；提供视图矩阵、投影矩阵与裁剪参数。
- Mesh.h：网格资源抽象；管理顶点/索引数据布局与绘制子集信息。
- Material.h：材质资源抽象；封装着色参数、纹理绑定与渲染开关。
- Texture.h：纹理资源抽象；管理像素格式、采样参数与 GPU 纹理对象生命周期。
- Shader.h：着色器程序抽象；管理编译结果、参数绑定与反射信息访问。
- ShaderManager.h：着色器管理器；负责着色器缓存、查找、复用与生命周期管理。
- RenderState.h：渲染状态描述；统一深度、混合、光栅化等 GPU 状态配置。

## 6. Runtime/Physics 头文件职责

### 6.1 模块分层

Physics 采用以下流水线：

1. 采样输入并同步 ECS 变换。
2. 固定步长积分。
3. 宽相位筛选潜在碰撞对。
4. 窄相位生成接触流形。
5. 约束与冲量解算。
6. 回写结果到 ECS。

### 6.2 每个头文件（类）预期职责

- World.h：兼容层/过渡入口，后续逐步收敛到 PhysicsWorld。
- PhysicsWorld.h：物理世界聚合根；管理刚体、碰撞体、重力、时间步与求解阶段调度。
- PhysicsSystem.h：ECS 系统入口；负责 PhysicsWorld 与 ECS 的双向同步和帧调度。
- RigidBody.h：刚体状态与动力学属性；质量、角动量、姿态四元数、惯量张量、力与力矩累积。
- Collider.h：碰撞体组件；关联实体、形状、物理材质、过滤掩码、触发器标记。
- CollisionShape.h：几何形状抽象；提供姿态感知 AABB 与支持函数（Support Mapping）。
- CollisionDetector.h：碰撞检测编排器；组织 BroadPhase 与 NarrowPhase 并产出接触数据。
- ContinuousCollision.h：连续碰撞检测模块；执行 TOI（Time Of Impact）搜索并驱动子步推进，避免高速穿透。
- ContactManifold.h：接触流形数据结构；保存接触点、法线、穿透深度、累计冲量。
- ContactSolver.h：接触求解模块；统一处理法向冲量、摩擦冲量、位置修正与 one-sided 接触策略。EPA 输出法线方向为 A→B，`EnsureClosingVelocity` 先用接触点速度判定接近，若接触点因旋转出现假分离则回退到质心线速度判定；对球体接触会约束冲量力臂为球半径，避免远离几何表面的接触点导致扭矩异常放大。
- PhysicsMaterial.h：接触材质参数；静摩擦、动摩擦、恢复系数、组合规则。
- Constraint.h：约束抽象；用于关节、距离、弹簧、关节限位等统一解算接口。
- Raycast.h：空间查询接口；支持射线、形状 sweep、重叠检测与过滤。
- Integrator.h：积分器接口；封装半隐式欧拉等积分策略。
- BroadPhase.h：宽相位接口；当前实现为动态 BVH（fat AABB + 增量插入/删除 + 移动重插入）生成潜在碰撞对；后续演进为静态八叉树（或网格）+ 动态 BVH 的混合宽相位。
- NarrowPhase.h：窄相位接口；当前实现为 GJK + EPA 生成接触法线与穿透深度。

## 7. Runtime/Platform 头文件职责

### 7.1 模块分层

Platform 采用以下平台抽象流程：

1. 创建平台窗口与图形上下文。
2. 处理系统消息与窗口事件。
3. 维护窗口尺寸、焦点与显示状态。
4. 提供平台能力查询与工具函数封装。
5. 向上层输出统一跨平台接口。

### 7.2 每个头文件（类）预期职责

- Window.h：窗口抽象；负责窗口创建销毁、事件轮询、交换链呈现与尺寸管理。
- PlatformUtils.h：平台工具集合；封装路径、环境、时钟、系统能力查询等跨平台辅助能力。

## 8. Runtime/Resource 头文件职责

### 8.1 模块分层

Resource 采用以下生命周期：

1. 资产标识与元数据查询。
2. 导入与格式转换。
3. 同步/异步加载。
4. 内存缓存与引用管理。
5. 文件变更监听与热重载。

### 8.2 每个头文件（类）预期职责

- FileSystem.h：文件系统抽象；统一虚拟路径、包体、平台文件 IO。
- ResourceManager.h：资源门面；对上层提供 load/get/release 接口。
- Handle.h：通用资源句柄（`Runtime::Handle`）；位于 `Runtime/include/Common/`，各子模块通过别名表达持有资源类型：Graphics 用 `MeshHandle`/`TextureHandle`，Resource 用 `AssetHandle`。
- AssetMetadata.h：资产元信息；资源类型、路径、依赖、导入配置、版本。
- Resource.h：资源基类；统一状态机、引用计数、内存占用信息。
- ResourceCache.h：资源缓存容器；命中查询、淘汰策略、容量控制。
- ResourceLoader.h：加载器基接口；定义同步/异步加载与错误返回协议。
- AssetDatabase.h：资产数据库；维护 GUID 到元数据映射和反向索引。
- ImportPipeline.h：导入流水线；负责源资产到运行时格式的转换与产物管理。
- TextureLoader.h：纹理加载器；支持贴图格式解析、mipmap、颜色空间约定。
- MeshLoader.h：网格加载器；解析顶点布局、子网格、骨骼/切线数据。
- MaterialLoader.h：材质加载器；解析材质参数并绑定纹理/着色器资源。
- ShaderLoader.h：着色器加载器；管理编译、缓存、变体 key 与反射数据。
- HotReloadWatcher.h：热更新监视器；监听文件变化并触发增量重载与依赖传播。

## 9. Runtime/Network 头文件职责

### 9.1 模块分层

Network 采用客户端-服务器架构，支持 TCP（可靠消息）与 UDP（实时状态），与 ECS 松耦合（独立线程，通过 EventBus 通信）。流水线如下：

1. 创建 Socket 并建立/监听连接。
2. 维护会话，处理身份认证与心跳。
3. 接收原始数据并通过 Channel 保证交付语义。
4. 反序列化消息并按类型分发到处理器。
5. 服务器广播权威状态快照。
6. 客户端预测输入并在收到服务器快照后回滚修正。
7. 通过 EventBus 将网络事件投递到 ECS。

### 9.2 每个头文件（类）预期职责

- NetworkManager.h：网络模块门面；管理连接池、会话、消息分发与模块生命周期。
- Socket.h：底层平台 Socket 封装；抽象 TCP/UDP 套接字创建、发送、接收与关闭。
- Connection.h：连接抽象；封装连接/断开/重连逻辑与连接状态机。
- Session.h：会话管理；跟踪客户端身份、认证状态、心跳与超时检测。
- Message.h：消息定义；包含消息头（类型 ID、序列号、长度）与载荷缓冲区。
- MessageSerializer.h：二进制序列化接口；将消息编码/解码为字节流，支持自定义类型注册。
- MessageDispatcher.h：消息路由；按消息类型 ID 分发到已注册的处理器回调。
- Channel.h：传输通道抽象；定义可靠/不可靠/有序交付语义，基于 UDP 实现可靠层。
- PacketBuffer.h：包缓冲管理；处理数据分片、重组、排序与流控。
- NetworkSystem.h：ECS 桥接层；作为独立模块通过 EventBus 与 ECS 松耦合通信，处理网络帧调度。
- StateSnapshot.h：状态快照；服务器端权威世界状态序列化、差异压缩与广播。
- ClientPredictor.h：客户端预测与回滚；缓存本地输入历史，在收到服务器快照后执行状态修正。

## 10. Runtime/UI 头文件职责

### 10.1 模块分层

UI 采用 ImGui 调试叠加层与自有保留模式 Widget 树双层架构，支持 Screen Space 与 World Space 渲染，使用锚点/绝对定位布局，复用 ECS EventBus 处理事件，支持简单单向数据绑定。流水线如下：

1. 构建或更新 Widget 树与 Canvas 层级。
2. 执行锚点布局计算，确定控件位置与尺寸。
3. 处理输入事件，沿 Widget 树命中测试并分发。
4. 将 UI 事件桥接到 ECS EventBus。
5. 触发单向数据绑定更新。
6. 收集可见控件并提交到 UIRenderer 批量绘制。
7. UIRenderer 按 Screen Space / World Space 分别渲染。
8. ImGui DebugUI 叠加层独立渲染调试信息。

### 10.2 每个头文件（类）预期职责

- UIManager.h：UI 模块门面；管理 Widget 树根节点、Canvas 集合、输入路由与渲染提交。
- UIRenderer.h：UI 渲染后端；批量收集 Widget 绘制数据并按 Screen/World Space 提交绘制调用。
- Widget.h：控件基类；定义通用属性（位置、尺寸、可见性、启用状态）、父子层级、事件处理虚接口。
- Canvas.h：画布根节点；定义渲染空间类型（Screen Space / World Space）、坐标系统与排序层级。
- UIEvent.h：UI 事件定义；点击、悬停、焦点、输入等事件类型，桥接到 ECS EventBus。
- UIStyle.h：样式/主题系统；颜色、字体、间距、边框等视觉属性集合与主题切换。
- Layout.h：布局引擎；基于锚点与绝对定位计算控件最终矩形，支持边距与对齐。
- DataBinding.h：单向数据绑定；属性观察者模式，数据源变化时自动更新关联控件。
- DebugUI.h：ImGui 集成层；封装 ImGui 初始化、帧提交与调试/编辑器叠加面板。

### 10.3 Widgets 子目录头文件职责

- Widgets/Label.h：文本标签控件；显示静态或绑定文本，支持字体、颜色、对齐设置。
- Widgets/Button.h：按钮控件；响应点击事件，支持普通/悬停/按下/禁用状态与样式。
- Widgets/Image.h：图片控件；显示纹理或图集精灵，支持 UV 裁剪与着色。
- Widgets/Panel.h：面板容器；作为 Widget 容器提供背景绘制、边框与子控件布局区域。
- Widgets/Slider.h：滑块控件；在指定范围内拖拽选值，支持水平/垂直方向。
- Widgets/ListView.h：列表视图；动态显示可滚动项目列表，支持项模板与选中回调。
- Widgets/ScrollView.h：滚动视图；为子内容提供可裁剪的可滚动区域与滚动条。
- Widgets/InputField.h：输入框控件；接受文本输入，支持占位符、光标与选中编辑。
- Widgets/Toggle.h：开关控件；布尔值切换，支持勾选框或滑动开关样式。
- Widgets/ProgressBar.h：进度条控件；显示 0–1 范围进度值，支持水平/垂直与样式。
- Widgets/TreeView.h：树视图控件；层级展开/折叠显示，支持节点选中与懒加载。
- Widgets/TabView.h：标签页控件；多页签切换容器，管理标签页与对应内容面板。
- Widgets/DropDown.h：下拉菜单控件；点击展开选项列表，支持搜索过滤与选中回调。
- Widgets/Dialog.h：对话框控件；模态/非模态弹窗，支持标题、内容区与按钮组。
