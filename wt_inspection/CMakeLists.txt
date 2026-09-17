cmake_minimum_required(VERSION 3.10)

project(wt_inspection C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# ---------------------------------------------------------------------------
# wt_core : 纯算法库，完全不依赖 PSDK，可在 PC 上直接编译与单元测试
#
# 这条分界线是刻意划的：规划、几何、安全校验这些"出错代价最大"的部分
# 必须能在桌面上反复验证；只有真正碰硬件的代码才进 wt_psdk。
# ---------------------------------------------------------------------------
add_library(wt_core STATIC
    src/wt_geometry.c
    src/wt_turbine.c
    src/wt_camera.c
    src/wt_plan.c
    src/wt_bridge.c
    src/wt_app_config.c
    src/wt_telemetry.c
    src/wt_runner.c
)
target_include_directories(wt_core PUBLIC include)
target_link_libraries(wt_core PUBLIC m)

# ---------------------------------------------------------------------------
# PC 侧演示与自检程序
# ---------------------------------------------------------------------------
add_executable(wt_plan_demo tools/wt_plan_demo.c)
target_link_libraries(wt_plan_demo PRIVATE wt_core)

# ---------------------------------------------------------------------------
# PC 侧回归测试
#
# 只为两条东西上锁：一条是「文档与自检程序里的数字必须由规划器自己产生」，
# 另一条是「配置里写下的参数必须真的生效」。两者都是曾经静默漂移过的地方，
# 靠人眼复核守不住。
#
#   cmake --build build && ctest --test-dir build --output-on-failure
#
# 交叉编译时不注册（tests/ 下的代码用不到，也不该进机载镜像）。
# ---------------------------------------------------------------------------
option(WT_BUILD_TESTS "编译并注册 PC 侧回归测试" ON)

if(WT_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    enable_testing()

    foreach(wt_test wt_test_plan wt_test_config wt_test_bridge)
        add_executable(${wt_test} tests/${wt_test}.c)
        target_include_directories(${wt_test} PRIVATE tests)
        target_link_libraries(${wt_test} PRIVATE wt_core)
        add_test(NAME ${wt_test} COMMAND ${wt_test})
        # 部分用例要写临时配置文件，给它们一个确定可写的工作目录
        set_tests_properties(${wt_test} PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endforeach()
endif()

# ---------------------------------------------------------------------------
# 机载目标：需要 PSDK 静态库
#
# 默认关闭。启用方式：
#   cmake -S . -B build-m3 -DWT_BUILD_PSDK_APP=ON \
#         -DPSDK_ROOT=<Payload-SDK 根目录>
#
# 目标架构由 -DWT_TARGET_ARCH 选择（默认 aarch64，即妙算3）：
#   aarch64        → psdk_lib/lib/aarch64-linux-gnu-gcc   （妙算3）
#   x86_64         → psdk_lib/lib/x86_64-linux-gnu-gcc    （PC 上做链接自检）
#
# 之所以留 x86_64 这条：应用层（授权信息是否合法、文件能否落盘、话题订阅
# 能否注册）里有相当一部分与 CPU 架构无关，在 PC 上编出来跑一次的代价远低于
# 把二进制拷到机子上再发现问题。它连不上飞机，但足以验证"程序能不能起来"。
# ---------------------------------------------------------------------------
option(WT_BUILD_PSDK_APP "编译 PSDK 机载应用（需要 PSDK 静态库）" OFF)
set(WT_TARGET_ARCH "aarch64" CACHE STRING "PSDK 目标架构：aarch64 | x86_64")

if(WT_BUILD_PSDK_APP)
    if(NOT PSDK_ROOT)
        message(FATAL_ERROR "需要指定 -DPSDK_ROOT=<Payload-SDK 根目录>")
    endif()

    if(WT_TARGET_ARCH STREQUAL "aarch64")
        set(WT_TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
        set(WT_ARCH_DEFINE PLATFORM_ARCH_aarch64=1)
    elseif(WT_TARGET_ARCH STREQUAL "x86_64")
        set(WT_TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
        set(WT_ARCH_DEFINE PLATFORM_ARCH_x86_64=1)
    else()
        message(FATAL_ERROR "不支持的 WT_TARGET_ARCH: ${WT_TARGET_ARCH}（可选 aarch64 | x86_64）")
    endif()

    set(WT_PSDK_LIB ${PSDK_ROOT}/psdk_lib/lib/${WT_TOOLCHAIN_NAME}/libpayloadsdk.a)

    if(NOT EXISTS ${WT_PSDK_LIB})
        message(FATAL_ERROR "找不到 PSDK 静态库: ${WT_PSDK_LIB}")
    endif()

    # 复用官方 manifold3 样例的平台适配层（hal + osal），不做重复移植
    set(WT_M3_DIR ${PSDK_ROOT}/samples/sample_c/platform/linux/manifold3)
    set(WT_COMMON_DIR ${PSDK_ROOT}/samples/sample_c/platform/linux/common)

    add_executable(wt_inspection_app
        app/main.c
        src/wt_telemetry_psdk.c
        src/wt_runner_psdk.c
        ${WT_M3_DIR}/hal/hal_usb_bulk.c
        ${WT_COMMON_DIR}/osal/osal.c
        ${WT_COMMON_DIR}/osal/osal_fs.c
        ${WT_COMMON_DIR}/osal/osal_socket.c
    )

    target_include_directories(wt_inspection_app PRIVATE
        include
        app
        ${WT_M3_DIR}/application
        ${WT_M3_DIR}/hal
        ${WT_COMMON_DIR}
        ${PSDK_ROOT}/psdk_lib/include
    )

    target_link_libraries(wt_inspection_app PRIVATE
        wt_core
        ${WT_PSDK_LIB}
        pthread
        m
        dl
    )

    target_compile_definitions(wt_inspection_app PRIVATE
        _GNU_SOURCE
        SYSTEM_ARCH_LINUX
        ${WT_ARCH_DEFINE}
        CONFIG_HARDWARE_CONNECTION=DJI_USE_ONLY_USB_BULK_DEVICE
    )

    set_target_properties(wt_inspection_app PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
endif()