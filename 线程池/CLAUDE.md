# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Scope and build layout

This directory contains two implementations of a C++ thread-pool exercise within the larger `cpp_projects` Git repository. The sibling `kdl` project is independent and has its own guidance.

- [MadeByAi/](MadeByAi/) is the runnable reference implementation. Its [CMakeLists.txt](MadeByAi/CMakeLists.txt) builds the `threadpool` static library, `threadpool_demo`, and optionally `threadpool_tests`.
- [MadeByHuman/](MadeByHuman/) is an unfinished, separate reimplementation of the queue and producer. It has no worker, executable, CMake target, or tests; the reference build does **not** compile or validate these files. Both versions declare a global `PushResult`, so their queue headers cannot simply be included together in one translation unit.

There is no CMake project at this directory's root. Run the commands below from this directory (`线程池/`), with `MadeByAi` explicitly selected as the source directory.

## Build and verification commands

Requirements: CMake 3.16+, a C++17 compiler, and platform thread support (linked through `Threads::Threads`). Tests are enabled by default and require an installed GoogleTest package discoverable by `find_package(GTest REQUIRED)`; CMake does not download it.

```sh
# Configure and build the library, demo, and tests.
cmake -S MadeByAi -B build
cmake --build build -j4

# Run the demo; inspect its printed PASS/FAIL summary.
./build/bin/threadpool_demo

# Run all tests, with a per-test timeout to catch hangs.
ctest --test-dir build --output-on-failure --timeout 30

# Run one test by its exact registered name.
ctest --test-dir build -R '^QueueTest\.CloseStillDrainsRemainingTasks$' --output-on-failure --timeout 30

# Run a suite directly through GoogleTest.
./build/bin/threadpool_tests --gtest_filter='QueueTest.*'

# Repeat the suite to exercise different thread schedules.
./build/bin/threadpool_tests --gtest_repeat=100 --gtest_brief=1
```

The default build type is `Release`; add `-DCMAKE_BUILD_TYPE=Debug` when configuring for debugging. Executables go into the selected build directory's `bin/` subdirectory. The demo always returns zero, even if its printed validation says `FAIL`, so exit status alone is not its correctness check.

To build without GoogleTest, use a separate configuration:

```sh
cmake -S MadeByAi -B build/no-tests -DBUILD_TESTING=OFF
cmake --build build/no-tests -j4
```

ThreadSanitizer is an existing CMake option:

```sh
cmake -S MadeByAi -B build/tsan -DSANITIZE_THREAD=ON
cmake --build build/tsan -j4
ctest --test-dir build/tsan --output-on-failure --timeout 30
```

TSan flags propagate from the library to the executables. Some container/WSL environments cannot run TSan and report `unexpected memory mapping`; this is not a passing sanitizer check. GoogleTest discovery executes the test binary during the build, so a runtime restriction can also fail the build step.

There is no dedicated lint or formatting target/configuration. The CMake targets enable `-Wall -Wextra`.

## Architecture and concurrency contracts

There is no owning `ThreadPool` facade. [main.cpp](MadeByAi/main.cpp) and the tests compose the pipeline directly:

`Generator → Producer_thread → shared Queue → Worker_thread → task invocation`

Tasks are `std::function<void()>`; a producer's `Generator` returns `std::optional<Task>`, with `nullopt` meaning the source is exhausted. Producers and workers borrow the queue by reference and own their respective `std::thread`. Construction does not start a thread; callers explicitly invoke `start()`. The queue and any objects captured by task/generator references must outlive their use by those threads.

### Queue and execution

- [Queue](MadeByAi/Queue.hpp) is a FIFO backed by `std::deque`, protected by one mutex and separate `not_full_` / `not_empty_` condition variables. Capacity zero means unbounded; positive capacity provides backpressure.
- Wait predicates include closure. Pushes wake consumers, pops wake producers, and `close()` wakes both groups. Helpers ending in `_locked` require the caller to hold the queue mutex.
- Closing rejects new pushes but preserves queued tasks. `wait_pop()` returns `nullopt` only when closed **and** empty; `try_pop()` also returns `nullopt` for a temporarily empty, open queue.
- Generators and task bodies execute outside the queue lock. Generator exceptions increment the producer's `failed()` count and are skipped; task exceptions increment the worker's `failed()` count and do not stop other tasks. `produced()` counts successful enqueue operations; `executed()` counts successful task invocations, not failures.

### Shutdown and important implementation limits

The lossless shutdown sequence for already-enqueued tasks is: let producers finish (or request their stop), join producers, call `Queue::close()`, then join workers. Joining an idle worker before closing its queue can block indefinitely. Joining a producer that fills a bounded queue without active consumers can also block indefinitely.

A worker's `request_stop()` is **not** equivalent to graceful draining: it closes the shared queue for every producer/worker and sets that worker's local stop flag, allowing it to exit before draining. Destructors call `request_stop()` and `join()`, so explicit coordinated shutdown matters.

[Producer_thread::run()](MadeByAi/Producer_thread.cpp) uses 50 ms enqueue waits to recheck its stop flag under backpressure. Currently a timed-out task is discarded and the next loop calls the generator again; it does **not** retry the same task. The timeout does not bound a blocking generator call. Do not infer stronger delivery or cancellation guarantees from the explanatory comments.

Producer and worker objects are neither copyable nor movable. The demo and tests use `std::deque` with `emplace_back` to avoid relocating running objects whose threads capture `this`.

## Tests and editor integration

[tests/test_threadpool.cpp](MadeByAi/tests/test_threadpool.cpp) contains the `QueueTest`, `ProducerTest`, `WorkerTest`, and `IntegrationTest` suites. `gtest_discover_tests` registers individual cases with CTest after building. Coverage includes FIFO/capacity/closure semantics, exception isolation, backpressure, lifecycle behavior, and a multi-producer/multi-worker exactly-once scenario. These tests cover only the AI implementation, not the human version or every timeout path.

CMake exports a compilation database and, on Unix, creates [MadeByAi/compile_commands.json](MadeByAi/compile_commands.json) as a symlink to the most recently configured build's database. Reconfiguring a different build directory repoints that link. The local [VS Code settings](.vscode/settings.json) select clangd, disable competing C/C++ IntelliSense, and explicitly point clangd at `${workspaceFolder}/build`; the primary build command above matches that layout when this directory is the workspace root. Build outputs, clangd caches, and generated compilation-database links are not source changes.
