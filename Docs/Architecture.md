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
│  │  ├─ AI/
│  │  ├─ Core/
│  │  ├─ ECS/
│  │  ├─ Gameplay/
│  │  ├─ Graphics/
│  │  ├─ Network/
│  │  ├─ Physics/
│  │  ├─ Platform/
│  │  ├─ Resource/
│  │  └─ UI/
│  └─ src/
│     ├─ AI/
│     ├─ Core/
│     ├─ ECS/
│     ├─ Gameplay/
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

- Engine.h：引擎总入口；负责 Window/Renderer 装配、生命周期管理与对上层启动接口。
- GameLoop.h：主循环编排器；组织逐帧可变步长更新、固定步长更新与渲染驱动（退出/关闭流程由 Engine 与 Game 层完成，GameLoop 自身无 shutdown 阶段）。
- Input.h：输入服务接口；统一键鼠状态采样、查询与帧内事件缓存（由 Window 的 GLFW 回调注入；不支持手柄）。
- Time.h：时间服务接口；提供 deltaTime、累计运行时间与 FPS 统计。固定步长累加器由 GameLoop 维护，Time 当前未接入 Engine 主循环。

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

- World.h：ECS 世界门面；聚合 Registry、SystemManager 与事件总线，Update 按注册顺序驱动系统。
- Registry.h：注册中心；负责实体与组件关系管理、单组件查询（EntitiesWith<T>）与实体销毁时的组件清理。
- Entity.h：实体句柄抽象；当前为 uint32_t 数值 ID，句柄本身不携带代际。
- EntityPool.h：实体池；负责实体 ID 分配、空闲列表复用与池内代际递增（销毁后复用的旧句柄不会失效，句柄级代际校验未实现）。
- Component.h：组件基抽象与类型标识；定义组件类型元信息与通用访问约定。
- ComponentPool.h：组件池；类型擦除接口 + 按实体 ID 索引的 vector<optional<T>> 存储（非紧凑连续存储）。
- System.h：系统基接口；定义 Update(Registry&, float) 更新接口。
- SystemPool.h：系统容器（SystemManager）；按注册顺序逐个更新系统，当前无阶段分组与启停状态。
- EventBus.h：事件总线；同步发布/订阅机制（Emit 直接在调用栈执行回调），无事件队列与异步分发。

说明：ECS 当前为头文件内联实现，Runtime/src/ECS 为空目录。

## 5. Runtime/Graphics 头文件职责

渲染后端为 OpenGL 3.3 Core：GLFW 创建窗口与上下文，GLAD 加载函数指针。当前管线 Execute 仅做 shader 绑定，实际绘制提交与 RenderState 应用集中在 Renderer::Flush。

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

### 5.3 ShadowMap/CSM 头文件架构（新增）

为支持方向光 ShadowMap 与 Cascaded Shadow Maps（CSM），Graphics 头文件设计了如下层次。当前实现状态：仅 Light.h 与 ShadowTypes.h 含有实际数据定义；其余 6 个头文件均为接口占位，没有对应 .cpp 实现，Runtime/src/Graphics/Shadows 目录不存在。

- Light.h：光源抽象与基础参数；至少包含方向光方向、颜色、强度、阴影开关。
- ShadowTypes.h：阴影公共数据结构；定义 `ShadowCascadeSettings`、`ShadowQualitySettings`、`ShadowFrameConstants`。
- ShadowMap.h：单张阴影贴图资源抽象；管理深度纹理、FBO、分辨率、采样状态与生命周期。
- ShadowAtlas.h：阴影图集/数组管理；统一管理多级级联贴图分配策略与布局信息。
- ShadowCulling.h：阴影可见性裁剪；根据光源与级联体积筛选投射体/接收体。
- ShadowRenderer.h：阴影渲染入口；组织阴影深度 Pass（Depth-only）并输出每级 LightViewProj。
- CascadedShadow.h：级联切分与稳定化策略；负责 Split 计算、级联包围体构建、Texel Snapping。
- LightingPass.h：主光照 Pass 阴影采样接口；执行阴影比较（Depth Compare）、PCF 过滤与级联混合。

### 5.4 ShadowMap/CSM 渲染流水线（新增）

以下为设计目标流程，当前未实现（无阴影 FBO、深度 Pass 与阴影采样着色器）：

1. 从主相机提取视锥参数并计算级联切分距离（CSM Split）。
2. 为每级级联构建 Light View/Projection，并做稳定化（Texel Snapping）。
3. 执行阴影深度 Pass：仅写深度，不输出颜色。
4. 在主光照 Pass 中根据像素深度选择级联并采样阴影图。
5. 执行 Bias（常量 + 斜率）与 PCF，输出阴影因子。
6. 在级联边界执行过渡混合，减少分层接缝。

### 5.5 与现有模块的边界约定（新增）

1. Renderer 负责帧级调度，不直接维护级联算法细节；级联逻辑下沉到 `CascadedShadow`。
2. Material/Shader 仅消费统一阴影常量，不感知贴图分配策略。
3. Resource 模块负责阴影相关 shader 与配置资产加载，不负责运行时阴影图创建。
4. Game 层仅通过 Light/Quality 配置阴影行为，不直接操作底层 Shadow FBO。

## 6. Runtime/Physics 头文件职责

### 6.1 模块分层

Physics 采用以下流水线：

1. 固定步长累加器驱动子步推进（ECS 输入同步与结果回写尚未接入，PhysicsSystem 仅转发 Step）。
2. 半隐式欧拉积分。
3. 宽相位（静态/动态双 BVH）筛选潜在碰撞对。
4. 中相位维护持久碰撞对与接触流形缓存。
5. 窄相位 GJK + EPA 生成接触点（支持多线程并行）。
6. 物理岛构建与冲量求解（独立岛可并行）。

### 6.2 每个头文件（类）预期职责

- World.h：兼容层/过渡入口，后续逐步收敛到 PhysicsWorld。
- PhysicsWorld.h：物理世界聚合根；管理刚体、碰撞体、重力、时间步与求解阶段调度。
- PhysicsSystem.h：ECS 系统入口；当前仅转发 PhysicsWorld 的固定步进 Step(dt)，PhysicsWorld 与 ECS 的双向同步尚未实现。
- RigidBody.h：刚体状态与动力学属性；质量、角动量、姿态四元数、惯量张量、力与力矩累积。
- Collider.h：碰撞体组件；关联实体、形状、物理材质、过滤掩码、触发器标记。
- CollisionShape.h：几何形状抽象；提供姿态感知 AABB 与支持函数（Support Mapping）。
- CollisionDetector.h：碰撞检测编排器；组织宽相/中相/窄相流程并产出接触数据，窄相通过共享 JobSystem 多线程并行分块执行。
- ContinuousCollision.h：连续碰撞检测模块；基于扫掠 AABB 的时间采样 + 二分细化近似 TOI，并驱动子步推进以避免高速穿透（非严格解析 TOI，且在调用线程逐对执行）。
- ContactManifold.h：接触流形数据结构；保存接触点、法线、穿透深度、累计冲量。
- ContactSolver.h：接触求解模块；统一处理法向冲量、摩擦冲量、位置修正与 one-sided 接触策略。EPA 输出法线方向为 A→B，`EnsureClosingVelocity` 先用接触点速度判定接近，若接触点因旋转出现假分离则回退到质心线速度判定；对球体接触会约束冲量力臂为球半径，避免远离几何表面的接触点导致扭矩异常放大。
- PhysicsMaterial.h：接触材质参数；静摩擦、动摩擦、恢复系数、组合规则（当前求解器仅使用 dynamicFriction，静摩擦未参与求解）。
- Constraint.h：约束抽象；当前仅有 Solve(float) 纯虚接口，关节、距离、弹簧、限位等具体约束类型尚未实现。
- Raycast.h：空间查询接口；当前仅实现基于 AABB slab 的射线检测（命中法线为 AABB 近似），形状 sweep 与重叠检测未实现。
- Integrator.h：积分器接口；封装半隐式欧拉等积分策略。
- BroadPhase.h：宽相位接口；当前实现为静态/动态双 BVH 树（fat AABB + 增量插入/删除 + refit 与移动重插入）生成潜在碰撞对。
- NarrowPhase.h：窄相位接口；当前实现为 GJK + EPA 生成接触法线、穿透深度与最多 4 点接触流形。
- BvhTree.h：动态 BVH 树实现；fat AABB、原地 refit、超出包围盒后 remove/reinsert。
- Midphase.h：中相持久对管理；PairKey/代际句柄槽位、持久接触流形、局部锚点匹配与累积冲量缓存，发布 Enter/Stay/Exit 接触事件。
- PhysicsIsland.h：物理岛构建与求解调度；独立岛支持并行求解，触发器不进入岛。
- PhysicsSettings.h：物理配置；并行窄相与并行岛求解的 worker 配置与开关。

## 7. Runtime/Platform 头文件职责

### 7.1 模块分层

Platform 采用以下平台抽象流程：

1. 创建平台窗口与图形上下文。
2. 处理系统消息与窗口事件。
3. 维护窗口尺寸、焦点与显示状态。
4. 提供平台能力查询与工具函数封装。
5. 向上层输出统一跨平台接口。

### 7.2 每个头文件（类）预期职责

- Window.h：窗口抽象；负责窗口创建销毁、事件轮询、交换链呈现与尺寸管理（基于 GLFW + OpenGL 3.3 Core 上下文）。
- PlatformUtils.h：平台工具集合；当前为占位实现（仅空的 EnableVSync），路径、环境、时钟、系统能力查询等能力尚未实现。

## 8. Runtime/Resource 头文件职责

### 8.1 模块分层

Resource 采用以下生命周期：

1. 资产标识与元数据查询。
2. 导入与格式转换。
3. 同步/异步加载。
4. 内存缓存与引用管理。
5. 文件变更监听与热重载。

### 8.2 每个头文件（类）预期职责

- FileSystem.h：文件系统抽象；挂载点管理、虚拟路径映射与文本/二进制读写（基于 std::filesystem，无包体系统）。
- ResourceManager.h：资源门面；对上层提供 load/get/release 接口，实现同步/异步加载流水线（IO → Decode → Upload）与缓存淘汰（基于 shared_ptr 引用计数判断，无独立的每资源状态机/显式引用计数）。
- Handle.h：通用资源句柄（`Runtime::Handle`）；位于 `Runtime/include/Common/`，各子模块通过别名表达持有资源类型：Graphics 用 `MeshHandle`/`TextureHandle`，Resource 用 `AssetHandle`。
- AssetMetadata.h：资产元信息；资源类型、路径、依赖、导入配置、版本。
- Resource.h：资源基类；统一状态枚举、内存占用信息。
- ResourceCache.h：资源缓存容器；命中查询、淘汰策略、容量控制。
- ResourceLoader.h：加载器基接口；定义 Read/Decode/Upload 加载协议。
- AssetDatabase.h：资产数据库；维护 GUID 到元数据映射和虚拟路径反向索引。
- ImportPipeline.h：导入流水线；当前仅生成 GUID 与构造元数据，源资产解析、格式转换与产物管理未实现。
- TextureLoader.h：纹理加载器；当前仅将文件包装为原始二进制资源，mipmap、颜色空间等解析未实现。
- MeshLoader.h：网格加载器；当前仅包装原始二进制数据，顶点布局、子网格、切线解析未实现。
- MaterialLoader.h：材质加载器；当前仅读取文本资源，材质参数解析与依赖绑定未实现。
- ShaderLoader.h：着色器加载器；当前仅读取文本资源，变体 key 与反射数据未实现。
- HotReloadWatcher.h：热更新监视器；当前仅基于修改时间轮询检测文件变化，增量重载与依赖传播未接入。

## 9. Runtime/Network 头文件职责

### 9.1 模块分层

Network 采用客户端-服务器架构，与 ECS 松耦合（独立线程，通过事件队列桥接）。流水线如下（其中第 3/5/6 步为规划能力，当前为接口占位，见 9.3）：

1. 创建 Socket 并建立/监听连接（TCP 已实现，UDP socket 已封装）。
2. 维护会话，处理心跳与超时检测。
3. 接收原始数据并通过 Channel 保证交付语义（未实现）。
4. 反序列化消息并按类型分发到处理器。
5. 服务器广播权威状态快照（未实现）。
6. 客户端预测输入并在收到服务器快照后回滚修正（未实现）。
7. 通过事件队列将网络事件投递到上层。

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

### 9.3 当前实现状态（2026-04-25）

第一阶段已落地（MVP）：

1. 已实现 TCP 端到端闭环：Socket/Connection/Message/Serializer/Dispatcher/PacketBuffer/NetworkManager 基础能力可用。
2. 已实现独立网络线程 + 线程安全事件队列，支持客户端连接、消息接收、心跳探活、断线后一次重连。
3. 已实现 Game 客户端示例接入：启动后连接 `127.0.0.1:45000`，周期发送 Ping，接收 Pong 并输出日志。
4. 已实现独立服务端示例 `LocalServerDemo`：监听本地端口并处理 Connect/Ping/Disconnect。
5. 已补齐 Network 测试并接入 CTest：序列化、包缓存、TCP 回环、断线重连。

第一阶段未实现（按分期策略保留）：

1. UDP 可靠层（Channel）仅保留接口占位，语义为第二阶段实现。
2. StateSnapshot 与 ClientPredictor 当前为数据结构/占位实现，未接入权威同步与回滚流程。

## 10. Runtime/UI 头文件职责

### 10.1 模块分层

UI 规划采用 ImGui 调试叠加层与自有保留模式 Widget 树双层架构，使用锚点布局与单向数据绑定。当前实现状态见 10.4；规划流水线如下：

1. 构建或更新 Widget 树与 Canvas 层级。
2. 执行锚点布局计算，确定控件位置与尺寸（对齐未实现）。
3. 处理输入事件，沿 Widget 树命中测试并分发。
4. 将 UI 事件桥接到 ECS EventBus（未接入）。
5. 触发单向数据绑定更新（控件自动绑定未实现）。
6. 收集可见控件并提交到 UIRenderer 批量绘制。
7. UIRenderer 按 Screen Space / World Space 分别渲染（未实现，仅按 layer 排序）。
8. ImGui DebugUI 叠加层独立渲染调试信息（未接入 ImGui）。

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

### 10.4 当前实现状态

1. UI 模块当前为头文件内联实现原型，Runtime/src/UI 下所有 .cpp 均为仅含 include 的空编译单元。
2. DebugUI 目前只是文本行缓存，尚未接入 ImGui 上下文与绘制；UIManager 仅写入两行统计文本。
3. UI 事件类型已定义，但尚未实际投递到 ECS EventBus（UIManager 仅保存指针）。
4. 样式系统仅有全局/类型样式表结构，运行时主题切换未实现，控件样式当前硬编码。
5. 布局支持锚点与边距，对齐（alignment）未实现；World Space Canvas 仅为数据标记，渲染后端未按空间分别处理。
6. 控件多为基础交互实现：ScrollView 与 Dialog 为最小骨架；ListView 无滚动/项模板；TreeView 命中仅处理根节点；DropDown 无搜索且展开项交互不完整；InputField 无光标/选中/删除编辑。

## 11. Runtime/Gameplay 头文件职责

### 11.1 模块分层

Gameplay 作为通用玩法运行时框架，采用以下流程：

1. 初始化实体级玩法状态（属性、标签、效果容器）。
2. 管理基础属性与修饰器叠加（Add/Multiply/Override）。
3. 应用与移除 GameplayEffect，并维护持续时间。
4. 通过标签引用计数处理状态门控与系统协作。
5. 每帧 Tick 清理过期效果并回收临时修饰。

### 11.2 每个头文件（类）预期职责

- GameplayTags.h：通用标签注册表与查询；维护标签名与运行时 ID 双向映射（引用计数门控位于 GameplaySystem 的实体级状态，不在注册表内）。
- AttributeSet.h：属性集容器；维护基础值、修饰器集合与最终值求解。
- GameplayEffect.h：效果规格与运行时实例；描述修饰器、时长、授予标签。
- GameplaySystem.h：玩法门面系统；按实体聚合属性/标签/效果并提供 Tick 驱动。

### 11.3 与 Game 层边界约定

1. Runtime/Gameplay 仅提供通用机制，不内置具体“遗物、天赋、塔技能”规则。
2. Game 层通过配置与脚本定义具体 EffectSpec 并调用 `GameplaySystem` 应用。
3. 标签命名规范由 Game 层维护，注册与匹配逻辑由 Runtime 层统一实现。

## 12. Runtime/AI/BehaviorTree 头文件职责

### 12.1 模块分层

AI 行为树采用“黑板 + 节点树 + 帧驱动”结构：

1. Blackboard 维护 AI 决策共享数据。
2. BehaviorTree 持有根节点并负责每帧 Tick 入口。
3. Composite 节点组织控制流（Sequence/Selector）。
4. Decorator 节点调整子树语义（Inverter/Repeat）。
5. Leaf 节点执行条件判断与动作回调。

### 12.2 每个头文件（类）预期职责

- Blackboard.h：黑板键值存储；提供类型安全读写与键管理。
- BehaviorNode.h：节点层级抽象；定义状态枚举、上下文、组合节点、装饰节点与叶节点接口。
- BehaviorTree.h：行为树运行入口；负责 root 装配、Tick 调度与重置。

### 12.3 与 ECS/Gameplay 边界约定

1. 行为树通过 `BehaviorContext` 持有实体 ID，不直接持有 World 生命周期。
2. 叶节点可通过 `userData` 注入 GameplaySystem/Navigation 等外部服务。
3. Blackboard 仅保存决策数据，不承担资源加载与持久化职责。
