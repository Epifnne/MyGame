# CI 故障排查实战手册（MyGame）

## 目标

这份文档总结一次完整的 CI 排障闭环：

1. 从“流水线红”快速定位到具体步骤。
2. 从“步骤失败”定位到可执行修复。
3. 每次只改最小范围并立即触发下一轮验证。
4. 沉淀可复用的排障路径，避免下次重复踩坑。

## 本次问题全景

本次是多因子叠加问题，不是单点故障：

1. 顶层 CMake 强制依赖 Qt，但当前项目并未用 Qt。
2. 依赖目录来自 git 子模块，CI 未初始化导致路径缺失。
3. Linux 缺少 wayland-scanner（GLFW 默认 Wayland）。
4. Windows 下 glad 代码生成依赖 Python 包 jinja2，解释器不一致导致未命中。
5. Windows 默认 shell 为 PowerShell，`ls -la` 语法不兼容。
6. 诊断步骤本身曾失败，掩盖了真实错误（`grep` 无匹配返回非 0）。

## 排查原则（建议固定执行）

1. 先定位失败步骤，不要猜根因。
2. 先让日志可观测，再优化流程。
3. 每次只改一个最可能根因。
4. 每次改动后立即 push 触发新 run。
5. 只在“已验证无关”后再扩大战术面。

## 标准排查路径

### 1) 看失败落点（步骤级）

先看是哪个步骤红：

1. `checkout` 失败：优先检查子模块和凭据。
2. `Configure` 失败：优先看 CMake 依赖、工具链、路径。
3. `Build` 失败：优先看编译器/链接器报错。
4. `Run tests` 失败：优先看测试输出与运行环境。

### 2) 让日志足够“可读”

如果错误信息不完整，先加日志收集：

1. `Configure` 失败时输出 `Build/CMakeFiles/CMakeError.log`、`CMakeOutput.log`、`configure.log` tail。
2. `Build` 失败时输出 `build.log` tail，并提取关键错误行。
3. 诊断步骤必须“不会因无匹配而失败”，否则会掩盖主因。

### 3) 锁定根因并最小修复

按本次真实链路举例：

1. Qt 报错：移除顶层强制 `find_package(Qt6 REQUIRED ...)`。
2. 缺失第三方目录：初始化必需子模块。
3. Linux `wayland-scanner` 缺失：关闭 GLFW Wayland 后端。
4. Windows `jinja2` 缺失：安装 jinja2，并把 CMake 的 `Python_EXECUTABLE` / `Python3_EXECUTABLE` 指向 setup-python 解释器。
5. PowerShell `ls -la` 报错：Windows 分支用 `Get-ChildItem`。

### 4) 立即验证（push -> 新 run）

每次修复都要做：

1. `git add`（只加本次相关文件）
2. `git commit`
3. `git push`
4. 观察新 run 的失败是否“前移”或“消失”

> 失败位置发生变化，通常说明你确实修掉了上一层问题。

## 本次高价值经验

1. 子模块工程里，`checkout` 成功不等于依赖目录可用，必须显式初始化必需子模块。
2. 只传 `Python3_EXECUTABLE` 不一定覆盖 `find_package(Python)`；很多第三方脚本读取的是 `Python_EXECUTABLE`。
3. Windows runner 默认 shell 是 PowerShell，Unix 命令参数不能直接照搬。
4. 诊断脚本本身要“容错”（例如 `grep ... || true`），否则会制造二次噪音。
5. CI 多平台矩阵里常见“Linux 已绿、Windows 单独红”，要分平台看。

## 可复用检查清单（下次直接套）

1. 失败步骤是 `checkout` / `configure` / `build` / `test` 哪一个？
2. 依赖是否来自子模块？是否初始化了必需子模块？
3. 是否有平台特异命令（bash 参数在 PowerShell 下不可用）？
4. 生成代码是否依赖 Python 包？解释器是否一致？
5. 日志采集步骤是否会因为空输出而失败？
6. 修复是否最小、可回滚、可复现？
7. 新 run 的失败位置是否发生变化（证明修复有效）？

## 建议保留的 CI 习惯

1. 工作流保留 `workflow_dispatch`，便于手动复测。
2. 配置/构建失败时输出 tail 注解，减少点进日志的往返。
3. 对第三方依赖明确“来源策略”：子模块、vendor、或在线拉取。
4. 对生成工具链（Python/CMake/编译器）尽量显式指定版本与路径。

## 一句话方法论

先让错误变清晰，再做最小修复，然后快速验证下一轮。反复迭代，直到红点收敛为 0。
