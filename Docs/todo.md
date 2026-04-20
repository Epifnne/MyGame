ECS模块：{
    能基本支持组件系统。
    
已完成：
    - 设计基础 ECS API（`Entity`, `Component`, `System` 抽象）。
    - 实现基本的 Entity 管理（创建、销毁、查询）。
    - 实现组件注册与访问机制。
    - 实现系统调度与执行框架。

已完成：
    - 添加示例组件（如 `TransformComponent`）与系统（如 `RenderSystem`）。
    - 添加单元测试覆盖 Entity 管理、组件访问、系统执行。

计划中:
    - 添加事件系统测试。
}


Graphics模块：{
    能支持默认shader和自定义shader。
    能支持默认渲染管线和自定义渲染管线。

已完成：
    - 设计基础 Graphics API（`Renderer`, `Shader`, `ShaderManager`, `RenderPipeline` 抽象）。
    - 实现默认内置 shader 与 `DefaultPipeline`，并在 `Renderer` 初始化时创建并绑定。

已完成：
    - 支持从文件/字符串加载自定义 shader 并注册到 `ShaderManager`。
    - 实现自定义渲染管线接口与示例管线（已添加 `SimplePipeline`，可使用已注册 shader）。
    - 渲染性能优化（批处理、渲染排序、资源复用）。
    - 添加 Demo 场景与单元测试，完成文档与使用示例。

已完成：
    - `ShaderManager::CreateFromFiles`：从文件加载并注册 shader。
    - 新增 `SimplePipeline`，通过 shader 名称使用已注册 shader。
    - 在 `Renderer` 中添加 `SetPipeline` 与 `LoadShaderFromFiles` API。

已完成 Demo:
    - 新增 `Demo` 可执行（`Demo/main.cpp`），绘制一个三角形，使用 `Runtime` 提供的 `Window` 与 `Renderer` 辅助代码。
    - Demo 着色器位于 `Demo/shaders/`。
    - Demo 的 CMake：`Demo/CMakeLists.txt`（将 Runtime 源直接编入 Demo 以简化集成）。

计划中:
    - 添加形变模块。
    - 添加骨骼绑定模块。
}


Physics模块：{
    能支持基本的物理模拟（如刚体、碰撞检测、简单的力学）。
    
已完成：
    - 设计基础 Physics API（`PhysicsWorld`, `RigidBody`, `Collider` 抽象）。
    - 实现基本的物理模拟功能（刚体运动、碰撞检测、简单力学）。
    - 添加示例场景与单元测试覆盖物理模拟。

计划中:
    - 调试demo球体形变。
}

resource管理模块：{

计划中 ：
    - 设计基础 Resource API（`ResourceManager`, `Resource` 抽象）。
    - 实现资源加载、缓存、访问机制。
    - 添加示例资源类型（如纹理、模型）与单元测试。
}

network模块：{
    客户端-服务器架构，支持 TCP+UDP，自定义二进制序列化，与 ECS 松耦合（通过 EventBus），支持状态同步（服务器权威+客户端预测/回滚）。

计划中：
    - 设计基础 Network API（`NetworkManager`, `Socket`, `Connection`, `Session`, `Message` 抽象）。
    - 实现底层 Socket 封装（TCP/UDP）与连接管理。
    - 实现自定义二进制消息序列化（`MessageSerializer`）。
    - 实现消息路由与分发机制（`MessageDispatcher`）。
    - 实现传输通道抽象（`Channel`），支持可靠/不可靠/有序交付。
    - 实现包缓冲与分片重组（`PacketBuffer`）。
    - 实现会话管理（`Session`），包括身份认证与心跳。
    - 实现 ECS 桥接层（`NetworkSystem`），通过 EventBus 松耦合通信。
    - 实现服务器端状态快照与差异广播（`StateSnapshot`）。
    - 实现客户端预测与回滚（`ClientPredictor`）。
    - 添加网络模块单元测试与集成测试。
}

UI模块：{
    ImGui 调试叠加层 + 自有保留模式 Widget 树游戏 UI，支持 Screen Space 与 World Space 渲染，锚点/绝对定位布局，复用 ECS EventBus，简单单向数据绑定，完整控件集。

计划中：
    - 设计基础 UI 框架（`UIManager`, `Widget`, `Canvas`, `UIRenderer` 抽象）。
    - 实现控件基类（`Widget`）与父子层级管理。
    - 实现锚点/绝对定位布局引擎（`Layout`）。
    - 实现 UI 事件系统（`UIEvent`），桥接到 ECS EventBus。
    - 实现样式/主题系统（`UIStyle`）。
    - 实现 Screen Space 与 World Space 画布（`Canvas`）。
    - 实现 UI 渲染后端（`UIRenderer`），批量绘制。
    - 实现单向数据绑定（`DataBinding`）。
    - 实现 ImGui 集成层（`DebugUI`）。
    - 实现基础控件集：Label, Button, Image, Panel, Slider。
    - 实现高级控件集：ListView, ScrollView, InputField, Toggle, ProgressBar。
    - 实现扩展控件集：TreeView, TabView, DropDown, Dialog。
    - 添加 UI 模块单元测试与示例场景。
}