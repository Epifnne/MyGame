# CD 思维模型与 Submodule 实战指南

这份文档面向发布流水线设计，目标不是“某一次跑通”，而是建立可复制、可排障、可演进的 CD 体系。

## 1. CD 思维模型（从目标倒推）

把发布当成一条稳定的生产线，按阶段拆解：

1. 触发 Trigger
- 什么时候发布：tag push、手动触发、主分支合并。
- 是否允许重跑同版本：例如同 tag 覆盖附件。

2. 输入 Input
- 代码版本（commit/tag）。
- 子模块/第三方依赖版本。
- 构建参数（Release/Debug、是否测试、平台矩阵）。

3. 环境 Environment
- 运行器（ubuntu-latest、windows-latest）。
- 工具链（编译器、CMake、Python）。
- 系统依赖（OpenGL/X11 等）。

4. 构建 Build
- Configure：cmake -S/-B 与关键变量。
- Compile：cmake --build。
- 多配置生成器--config Release会导致嵌套release目录，输出路径要注意。
- 验证：ctest。

5. 组包 Package
- 统一产物目录结构（如 release/bin + release/assets）。
- 自动探测输出路径，不要写死单路径。
- 失败时给出可读日志（列目录、打印候选路径）。

6. 发布 Release
- 上传 artifact。
- 创建/更新 GitHub Release。
- 附件命名带 tag 与平台。

7. 回溯与可观测 Observability
- 失败日志要能直达根因。
- 每阶段输出关键上下文（解释器路径、候选输出目录、tag 来源）。

## 2. 一套可迁移的 CD 设计准则

1. 可重入
- 同一版本允许重跑并覆盖附件（例如 overwrite_files: true）。

2. 可复现
- 固定关键版本和输入（工具链、依赖、tag）。

3. 可定位
- 每一步失败都能看出“缺依赖、路径错、命令错、权限错”。

4. 最小假设
- 不假设构建输出路径唯一，使用候选路径探测。

5. 平台对齐
- Linux 与 Windows 结构一致，只在 shell 语法层面差异化。

## 3. Shell 设计原则（bash 与 pwsh）

1. 先定义变量，再执行动作
- 先计算 tag、python 路径、stage 目录，再 build/package。

2. 先检查，再复制
- 先 Test-Path 或目录存在判断，再 Copy。

3. 找不到就明确失败
- bash 用 exit 1，pwsh 用 throw。

4. 打印诊断信息
- 包含候选路径、目录树片段、关键变量值。

## 4. 为什么项目常用 Submodule

1. 版本可控
- 主仓库能精确锁定每个第三方库的 commit，避免“今天能过明天挂”。

2. 变更可审计
- 三方库升级在 PR 中是显式变更，方便 Code Review。

3. 降低供应链波动
- 不依赖每次构建在线拉最新包，减少外部源波动风险。

4. 多平台一致
- CI、本地、不同开发机都基于同一依赖快照。

## 5. Submodule 的代价与冲突来源

常见冲突本质：

1. 不同分支把同一个子模块指向了不同 commit。
2. 上游三方库接口变更，主工程代码未同步适配。
3. 子模块 URL、分支策略不一致（.gitmodules 变更）。

