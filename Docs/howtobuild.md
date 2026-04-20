# How To Build (Terminal First)

本文档面向本仓库内的 agent / 开发者，约定使用终端进行构建，不依赖 IDE 按钮。

## 适用范围

- 仓库根目录：`E:/MyGame`
- 平台：Windows
- 推荐工具链：MinGW（Qt 自带）

## 为什么优先用终端

当前环境下，CMake Tools 可能默认尝试 `NMake Makefiles`，如果系统没有可用的 MSVC/NMake，会出现“无法配置项目”这类泛化错误。

终端直接执行 `cmake -S -B` 可以拿到完整错误，并可显式指定生成器和编译器。

## 一次性配置（MinGW）

在仓库根目录执行：

```powershell
cmake -S . -B Build -G "MinGW Makefiles" ^
  -DCMAKE_C_COMPILER="D:/Qt/Tools/mingw1310_64/bin/gcc.exe" ^
  -DCMAKE_CXX_COMPILER="D:/Qt/Tools/mingw1310_64/bin/g++.exe"
```

说明：
- 如果你已经在 `Build` 目录用过其他生成器（如 NMake），先清理 `Build/CMakeCache.txt` 和 `Build/CMakeFiles`，再按本文命令重新配置。
- 本项目统一使用 `Build` 作为构建目录。

## 日常构建

```powershell
cmake --build Build -j 4
```

成功标志（示例）：
- `Built target MyGame`
- 日志出现 `Copying Game assets to runtime output directory`

## 运行前检查

构建后应至少确认：

- 可执行文件：`MyGame/MyGame.exe`
- 资源目录：`MyGame/assets`
- 着色器目录：`MyGame/assets/shaders`
- 静态库目录：`MyGame/lib`
- 测试程序目录：`MyGame/tests`

> 本仓库已在顶层 CMake 中添加 `POST_BUILD` 资源复制逻辑，构建 `MyGame` 后会自动把 `Game/assets` 复制到运行目录下。

## 安装（发布）

如果需要执行安装阶段（将产物安装到 `CMAKE_INSTALL_PREFIX` 指定目录）：

```powershell
cmake --install Build
```

说明：
- 仅 `--build` 不会自动触发 `--install`。
- 本仓库已添加 assets 的 install 规则，安装时会带上资源目录。


## 测试

如果开启了测试（`BUILD_TESTING=ON`），可执行：

```powershell
ctest --test-dir Build --output-on-failure
```

## 常见问题

1. 报错：生成器不匹配（MinGW vs NMake）
- 原因：同一个构建目录曾被其他生成器初始化。
- 处理：统一使用 `Build`，若发生冲突先清理 `Build/CMakeCache.txt` 和 `Build/CMakeFiles` 后重新配置。

2. 报错：`nmake` not found / 编译器未设置
- 原因：当前 shell 没有 MSVC 环境。
- 处理：改用本文 MinGW 命令，显式指定 `gcc/g++`。

3. 资源路径找不到
- 先确认日志中有资源复制语句。
- 再检查 `MyGame/assets` 是否存在。

4. glad 阶段看起来在“联网下载”
- 现象：日志里出现 `getting 'gl' specification from remote location`。
- 原因：`glad_add_library(...)` 是“构建时代码生成”，不是直接链接一个预生成静态库。
- 当前仓库做法：`Runtime/CMakeLists.txt` 已使用 `REPRODUCIBLE`，优先使用内置规范生成。
- 说明：日志里仍可能看到 remote URL，但若出现 `intercepted attempt to retrieve resource`，表示访问被拦截并走本地资源，不是实际外网依赖。

5. glad 使用 `REPRODUCIBLE` 后可离线稳定生成，构建可达 `Built target MyGame`。


## 给其他 Agent 的执行约定

1. 默认在仓库根目录执行命令。
2. 默认使用 `Build`。
3. 不要在同一构建目录切换生成器。
4. 先 `configure`，再 `build`，需要发布时再 `install`。
5. 出现构建问题时，优先回传完整终端错误，不要只回传“配置失败”。
