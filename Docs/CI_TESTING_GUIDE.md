# CI/CD 与测试脚本落地说明

## 目录建议

- 业务代码：`Runtime/`、`Game/`
- 单元测试：`Tests/`
- 工作流：`.github/workflows/`

当前项目已采用：

- `Tests/CMakeLists.txt`：使用仓库内 `ThirdParty/googletest`（随仓库版本管理），并由 `gtest_discover_tests` 自动注册到 CTest
- `Tests/ECS/RuntimeECS_SmokeTest.cpp`：ECS 单元测试样例（gtest）
- `Tests/Graphics/Graphics_BasicTest.cpp`：Graphics 单元测试样例（gtest）
- `.github/workflows/ci.yml`：构建并通过 `ctest` 运行测试

## 如何编写测试脚本

在 C++ 项目里，测试脚本通常就是测试源文件（`.cpp`）+ CMake 测试注册。

1. 在 `Tests/` 下新增 `XXX_Test.cpp`
2. 在 `Tests/CMakeLists.txt` 里把它加入一个可执行文件并链接 `GTest::gtest_main`
3. 用 `gtest_discover_tests(目标名)` 自动注册到 CTest
4. CI 使用 `ctest --test-dir Build --output-on-failure -C Release` 自动执行

示例（已落地）：

- 源文件：`Tests/RuntimeECS_SmokeTest.cpp`
- 源文件：`Tests/Graphics/Graphics_BasicTest.cpp`
- 注册：`Tests/CMakeLists.txt`

## 本地运行

```bash
cmake -S . -B Build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build Build --config Release -- -j
ctest --test-dir Build --output-on-failure -C Release
```

## CI/CD 分层建议

- CI（每次 push/PR）：编译 + 自动测试（当前 `ci.yml`）
- CD（打 tag）：构建产物 + 运行测试 + 发布 Release（`release.yml`）

当前 `release.yml` 已包含：

1. 开启测试构建（`-DBUILD_TESTING=ON`）
2. 运行 `ctest`
3. 打包 `Bin` 到 zip
4. 上传 zip 到 GitHub Release
