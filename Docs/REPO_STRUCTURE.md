# 仓库结构与版本管理建议

目标：清晰目录分工，便于维护与发布，使用语义化版本（SemVer）。

1. 目录结构（建议）

- /Game - 游戏代码（游戏逻辑、资源引用）
- /Runtime - 引擎运行时、平台抽象与子系统实现
- /Shared - 可复用库与工具（供 Game/Runtime/Tools 复用）
- /ThirdParty - 第三方依赖（源码或子模块）
- /Tools - 开发工具、编辑器扩展、测试与示例
- /Docs - 文档、架构说明、仓库规范
- /Bin - 构建产物（忽略到 .gitignore）
- /Build - CMake 构建目录（通常在 .gitignore 中）

2. 版本管理策略

- 使用语义化版本：MAJOR.MINOR.PATCH（例如 1.2.3）
- 主分支：`main`（始终为可发布的稳定状态）
- 开发分支：`develop`（整合日常开发）
- 功能分支：`feature/*`（从 `develop` 发起，完成后合并回 `develop`）
- 发布分支：`release/*`（用于准备发布，合并到 `main` 并打标签）
- 热修复分支：`hotfix/*`（从 `main` 发起，修复后合并回 `main` 和 `develop`）

3. 发布与打标签

- 在准备发布时，从 `develop` 创建 `release/x.y`，完成后合并到 `main`。
- 在 `main` 上通过 git tag 创建版本标签：`vMAJOR.MINOR.PATCH`。
- 将 `VERSION` 文件与 CMake 项目版本同步（手动或通过 CI 脚本）。

4. CI/CD 建议

- 在 CI 中验证：构建、单元测试、静态分析。
- 发布流程：当在 `main` 推送带 `v*` 标签时，CI 可自动打包并发布二进制。

5. 其他建议

- 添加或更新 `.gitignore`，排除 `Bin/`、`Build/`、临时文件、IDE 配置等。
- 在 `Docs/` 中维护 `CONTRIBUTING.md`、`RELEASE_PROCESS.md`（可选）。
