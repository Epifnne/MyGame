# 未来构想：Scripting 模块与 JSX UI 布局

> 状态：构想阶段，不在当前档期实施。

## 1. 目标

- 使用编译型 JS（QuickJS）作为业务脚本层。
- 使用 JSX 声明式语法在 JS 中定义 UI 布局。
- 支持 TypeScript 和纯 JavaScript。
- 支持脚本与 UI 布局的热重载。

## 2. 技术选型

| 项目 | 选择 | 理由 |
|------|------|------|
| JS 引擎 | QuickJS | 轻量、C API、编译为字节码、MIT 协议、游戏引擎常用 |
| UI 描述方式 | JSX 声明式 | 类似 React，直观表达层级结构，转译后为普通函数调用 |
| 转译工具 | esbuild / swc | 极快（毫秒级），支持 TSX/JSX → JS |
| 类型系统 | TypeScript 为主，JS 也支持 | TS 提供类型安全与 IDE 补全 |

## 3. 整体流水线

```
.tsx/.jsx ──→ esbuild/swc 转译 ──→ .js ──→ QuickJS 编译 ──→ 字节码 ──→ 引擎加载执行
                                                                          │
                              ┌───────────────────────────────────────────┘
                              ▼
                   JS 调用 createElement() 返回 VNode 树
                              │
                              ▼
                   C++ Reconciler 对比差异
                              │
                              ▼
                   更新 Widget 树（复用 Runtime/UI 模块）
```

JSX 只是语法糖，`<Panel><Button text="OK"/></Panel>` 转译后变成：
```js
createElement(Panel, null, createElement(Button, { text: "OK" }))
```

## 4. Runtime/Scripting 模块设计

### 4.1 模块分层

Scripting 采用以下流水线：

1. 初始化 QuickJS Runtime 与默认 Context。
2. 注册 C++ 原生绑定（ECS、Physics、Graphics、Network、UI 等引擎 API）。
3. 加载 ES Module，解析 import 依赖并编译。
4. 每帧由 ScriptSystem 通过 EventBus 驱动脚本 `update(dt)` 回调。
5. 文件变更时触发热重载：转译 → 重编译 → 替换模块 → 重新执行。

### 4.2 每个头文件（类）预期职责

- ScriptEngine.h：QuickJS 引擎生命周期管理；创建/销毁全局 Runtime 与默认 Context。
- ScriptContext.h：隔离执行上下文；管理全局对象、模块加载器、异常捕获。
- ScriptValue.h：JS 值的 C++ 类型安全包装；自动引用计数、类型转换辅助。
- NativeBinding.h：C++ 类/函数注册框架；将引擎 API 暴露给 JS 调用。
- ScriptModule.h：ES Module 加载器；解析 import、按路径查找并编译模块。
- ScriptSystem.h：ECS 桥接系统；每帧调用脚本回调，通过 EventBus 与 ECS 松耦合。
- ScriptHotReloader.h：文件监听与脚本热重载；检测变更、重新编译、替换 Context 中的模块。

## 5. JSX → Widget 桥接设计

### 5.1 流水线

```
文件变更(.tsx) → HotReloadWatcher 检测 → esbuild 转译 → QuickJS 重新编译
    → ScriptHotReloader 替换模块 → UIBridge 重新 reconcile → Widget 树更新
```

### 5.2 桥接层头文件职责

- JSXRuntime.h：实现 `createElement` / `Fragment`；在 JS 侧构建轻量 VNode 描述树。
- UIBridge.h：C++ 侧取回 VNode 树，与现有 Widget 树做 diff/reconcile，增量更新。

### 5.3 转译配置

esbuild / swc 配置：
```json
{
  "jsxFactory": "createElement",
  "jsxFragment": "Fragment"
}
```

## 6. 预期目录结构

```text
Runtime/
├─ include/
│  └─ Scripting/
│     ├─ ScriptEngine.h
│     ├─ ScriptContext.h
│     ├─ ScriptValue.h
│     ├─ NativeBinding.h
│     ├─ ScriptModule.h
│     ├─ ScriptSystem.h
│     ├─ ScriptHotReloader.h
│     ├─ JSXRuntime.h
│     └─ UIBridge.h
└─ src/
   └─ Scripting/
      ├─ ScriptEngine.cpp
      ├─ ScriptContext.cpp
      ├─ ScriptValue.cpp
      ├─ NativeBinding.cpp
      ├─ ScriptModule.cpp
      ├─ ScriptSystem.cpp
      ├─ ScriptHotReloader.cpp
      ├─ JSXRuntime.cpp
      └─ UIBridge.cpp
```

## 7. 第三方依赖

- QuickJS：添加到 `ThirdParty/quickjs/`（源码编译，约 5 个 .c 文件）。
- esbuild / swc：作为外部工具链，开发时通过 npm 安装，不编入引擎。

## 8. 与现有模块的关系

- **UI 模块**：Scripting 的 JSX 桥接层复用 `Runtime/UI` 的 Widget 树，不替代它。
- **ECS**：ScriptSystem 通过 EventBus 松耦合接入，与 NetworkSystem 模式一致。
- **Resource**：脚本文件（.js/.qjsc 字节码）作为资源类型纳入 ResourceManager 管理。
- **HotReloadWatcher**：复用 Resource 模块已有的文件监听基础设施。
