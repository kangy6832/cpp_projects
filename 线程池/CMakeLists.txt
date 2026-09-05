cmake_minimum_required(VERSION 3.16)
project(thread_pool LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Threads REQUIRED)

add_library(threadpool STATIC
    Queue.cpp
    Producer_thread.cpp
    Worker_thread.cpp
)
target_include_directories(threadpool PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(threadpool PUBLIC Threads::Threads)
target_compile_options(threadpool PRIVATE -Wall -Wextra)

add_executable(threadpool_demo main.cpp)
target_link_libraries(threadpool_demo PRIVATE threadpool)
target_compile_options(threadpool_demo PRIVATE -Wall -Wextra)

# 单元测试：cmake -B build && cmake --build build && ctest --test-dir build --output-on-failure
option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(threadpool_tests tests/test_threadpool.cpp)
    target_link_libraries(threadpool_tests PRIVATE threadpool GTest::gtest GTest::gtest_main)
    target_compile_options(threadpool_tests PRIVATE -Wall -Wextra)

    include(GoogleTest)
    gtest_discover_tests(threadpool_tests)
endif()

# 线程安全检测：cmake -DSANITIZE_THREAD=ON .. && ./bin/threadpool_demo
option(SANITIZE_THREAD "Build with ThreadSanitizer" OFF)
if(SANITIZE_THREAD)
    target_compile_options(threadpool PUBLIC -fsanitize=thread -g -O1)
    target_link_options(threadpool PUBLIC -fsanitize=thread)
endif()
