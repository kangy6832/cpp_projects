# ============================================================================
#  CMakeLists.txt —— 构建配置
# ============================================================================
#
#  常用命令：
#    构建并运行演示程序
#      cmake -B build && cmake --build build -j4 && ./build/bin/threadpool_demo
#    构建并运行单元测试（ctest 会逐个用例跑，失败时打印详情）
#      cmake -B build && cmake --build build -j4
#      ctest --test-dir build --output-on-failure
#    关闭测试构建
#      cmake -B build -DBUILD_TESTING=OFF
#    开启 ThreadSanitizer（检测数据竞争）
#      cmake -B build-tsan -DSANITIZE_THREAD=ON && cmake --build build-tsan -j4
#
# ============================================================================

cmake_minimum_required(VERSION 3.16)
project(thread_pool LANGUAGES CXX)

# 【clangd】生成 compile_commands.json（编译数据库），clangd 靠它拿到
# 每个源文件的真实编译命令（宏定义、include 路径、-std=c++17 等），
# 否则它会瞎猜，报一堆 "file not found" / 找不到 std::optional 之类的假错。
# 注意：1) 必须在任何 add_library/add_executable 之前设置，否则不生效；
#       2) 文件生成在构建目录（build/compile_commands.json）。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# clangd 默认从「源文件所在目录往上逐级」找 compile_commands.json，
# 而它生成在 build/ 里，所以在源码根目录建个软链接指过去，
# 这样 clangd 开箱即用，不必手动加 --compile-commands-dir=build。
if(UNIX AND NOT CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json)
endif()

# 本项目用到 std::optional、std::shared_mutex 等特性，需要 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)   # 编译器不支持 C++17 就直接报错，而不是悄悄降级
set(CMAKE_CXX_EXTENSIONS OFF)         # 关闭 -std=gnu++17 的 GNU 扩展，保证可移植性

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 统一把可执行文件放到 build/bin，避免和库文件混在一起
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 【踩坑】Linux 下用 std::thread 必须链接 pthread 库。
# 直接写 -lpthread 不够可移植（macOS/Windows 不同），
# 用 CMake 的 Threads::Threads 导入目标可以自动适配各平台。
find_package(Threads REQUIRED)

# ---------------- 核心库 ----------------
add_library(threadpool STATIC
    Queue.cpp
    Producer_thread.cpp
    Worker_thread.cpp
)

# PUBLIC：链接 threadpool 的目标（demo / tests）会自动继承这个头文件搜索路径
target_include_directories(threadpool PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# PUBLIC：链接者自动继承 pthread。
# 注意仅 find_package 找到的导入目标才能这样写，比手写 -lpthread 可靠得多。
target_link_libraries(threadpool PUBLIC Threads::Threads)

# -Wall -Wextra 打开常用警告。
# 并发代码里很多隐患（如未初始化变量、有符号无符号比较）都会被它们抓到。
target_compile_options(threadpool PRIVATE -Wall -Wextra)

# ---------------- 演示程序 ----------------
add_executable(threadpool_demo main.cpp)
target_link_libraries(threadpool_demo PRIVATE threadpool)
target_compile_options(threadpool_demo PRIVATE -Wall -Wextra)

# ---------------- 单元测试 ----------------
option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)
    enable_testing()   # 必须调用，否则 ctest 找不到任何测试
    find_package(GTest REQUIRED)

    add_executable(threadpool_tests tests/test_threadpool.cpp)
    target_link_libraries(threadpool_tests PRIVATE
        threadpool
        GTest::gtest
        GTest::gtest_main   # 提供 main()，测试文件里就不必自己写了
    )
    target_compile_options(threadpool_tests PRIVATE -Wall -Wextra)

    # gtest_discover_tests 在构建后扫描可执行文件里注册了哪些 TEST()，
    # 把它们逐个登记给 ctest，于是 ctest 能单独运行/过滤每个用例。
    include(GoogleTest)
    gtest_discover_tests(threadpool_tests)
endif()

# ---------------- ThreadSanitizer ----------------
# 原理：在编译期插桩，运行时记录每次内存访问与同步操作，
# 用 happens-before 算法找出没有同步保护的内存访问（数据竞争）。
#
# 【注意】TSan 需要 -O1 及以上且保留调试信息，否则报告里没有行号。
# 【注意】在部分容器/WSL 环境里 TSan 会因内存映射受限而无法运行
#        （报 "unexpected memory mapping"），换物理机或关闭 ASLR 后可正常执行。
option(SANITIZE_THREAD "Build with ThreadSanitizer" OFF)
if(SANITIZE_THREAD)
    # 用 PUBLIC：让 demo 和 tests 也自动带上 sanitize 选项
    target_compile_options(threadpool PUBLIC -fsanitize=thread -g -O1)
    target_link_options(threadpool PUBLIC -fsanitize=thread)
endif()
