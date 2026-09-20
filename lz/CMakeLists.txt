cmake_minimum_required(VERSION 3.10)

project(liangzhourenwu C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

# ---------------------------------------------------------------------------
# 目标分层（依赖严格单向，见 include/lz_types.h 的说明）
#
#   lz_core    纯算法，不依赖 PSDK / 任何第三方      ← 永远可编译
#   lz_vision  视觉（纯 C，也是零依赖）             ← 永远可编译
#   lz_app     机载应用，依赖 PSDK                  ← 可选
#
# 分界线的理由：规划与安全校验出错代价最大，必须能在桌面上反复验证。
# PSDK 很重（要接飞机），不该成为跑测试的前提。视觉单独成层是因为它
# 改动频繁，不该拖着规划层的测试一起翻车。
# ---------------------------------------------------------------------------

# ---- lz_core：纯算法，无外部依赖 ----
add_library(lz_core STATIC
    src/lz_types.c
    src/lz_target.c
    src/lz_geo.c
    src/lz_plan.c
    src/lz_bridge.c
    src/lz_wpml.c
    src/lz_kmz.c
)
target_include_directories(lz_core PUBLIC include)
target_link_libraries(lz_core PUBLIC m)

# ---- 桌面演示工具 ----
#
# 受开关控制：设备上编译时 tools/ 目录往往没打包进去（只需要 app/ 与 src/），
# 若这里无条件 add_executable，CMake 会因为找不到源文件而**配置失败** ——
# 报的是 "No SOURCES given to target"，但真正的原因是目录没传上去，
# 提示与病因不匹配。
option(LZ_BUILD_DESKTOP_TOOLS "编译桌面演示工具（需要 tools/ 目录）" ON)

if(LZ_BUILD_DESKTOP_TOOLS)
    add_executable(lz_plan_demo tools/lz_plan_demo.c)
    target_link_libraries(lz_plan_demo PRIVATE lz_core)
    add_executable(lz_kmz_demo tools/lz_kmz_demo.c)
    target_link_libraries(lz_kmz_demo PRIVATE lz_core)
endif()

# ---- PC 侧回归测试 ----
option(LZ_BUILD_TESTS "编译并注册 PC 侧回归测试" ON)

if(LZ_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    foreach(lz_test lz_test_geo lz_test_plan lz_test_bridge lz_test_kmz)
        add_executable(${lz_test} tests/${lz_test}.c)
        target_include_directories(${lz_test} PRIVATE tests)
        target_link_libraries(${lz_test} PRIVATE lz_core)
        add_test(NAME ${lz_test} COMMAND ${lz_test})
        set_tests_properties(${lz_test} PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endforeach()
endif()

# ---------------------------------------------------------------------------
# lz_vision：视觉层（**纯 C，零依赖**）
#
# 刻意不用 OpenCV：国旗是高饱和红色块，HSV 双区间阈值 + 连通域足够，
# 纯 C 约 200–400 行。保持零依赖有两层好处：
#   1. WSL 与妙算3 都不必装 OpenCV（设备上有没有尚未确认）
#   2. 与 lz_core 一样能在桌面上直接跑回归测试
#
# 它与 lz_core **并列**而非其一部分：lz_core 是"永远可编译"的底线，
# 视觉算法改动频繁，不该拖着规划层的测试一起翻车。
# 依赖方向仍是单向：lz_vision → lz_core（用它的 LzTarget/几何），反向没有。
#
# 两个后端**互斥**（都定义 LzVision_Detect，同时编会符号冲突）：
#   hsv  —— src/lz_vision.c，真算法。连通域部分尚未实现，Detect 返回 UNSUPPORTED
#   stub —— src/lz_vision_stub.c，固定杆位占位。**当前默认**，
#           目的是让「拨开关 → 飞机绕一圈」这条链路先能整体跑通
# 切回真算法：-DLZ_VISION_BACKEND=hsv
# ---------------------------------------------------------------------------
set(LZ_VISION_BACKEND "stub" CACHE STRING "视觉后端：stub | hsv")

if(LZ_VISION_BACKEND STREQUAL "stub")
    set(LZ_VISION_SRC src/lz_vision_stub.c)
elseif(LZ_VISION_BACKEND STREQUAL "hsv")
    set(LZ_VISION_SRC src/lz_vision.c)
else()
    message(FATAL_ERROR "不支持的 LZ_VISION_BACKEND: ${LZ_VISION_BACKEND}（可选 stub | hsv）")
endif()

add_library(lz_vision STATIC ${LZ_VISION_SRC})
target_include_directories(lz_vision PUBLIC include)
target_link_libraries(lz_vision PUBLIC lz_core m)

# ---------------------------------------------------------------------------
# lz_app：机载目标，需要 PSDK 静态库
#
#   cmake -S lz -B build-native -DLZ_BUILD_PSDK_APP=ON \
#         -DPSDK_ROOT=<Payload-SDK 根目录>
#
# 目标架构由 -DLZ_TARGET_ARCH 选择（默认 aarch64，即妙算3）：
#   aarch64 → psdk_lib/lib/aarch64-linux-gnu-gcc
#   x86_64  → psdk_lib/lib/x86_64-linux-gnu-gcc   （PC 上做链接自检）
#
# 留 x86_64 这条的理由同 wt_inspection：应用层有一半的逻辑（配置读取、
# 文件落盘、订阅注册）与 CPU 架构无关，在 PC 上编出来跑一次的成本远低于
# 拷到机子上才发现问题。
# ---------------------------------------------------------------------------
option(LZ_BUILD_PSDK_APP "编译 PSDK 机载应用（需要 PSDK 静态库）" OFF)
set(LZ_TARGET_ARCH "aarch64" CACHE STRING "PSDK 目标架构：aarch64 | x86_64")

option(LZ_POLE_SOURCE_LASER "绕飞圆心取自激光测距（需 GPS 锁定且激光打中杆）" OFF)

if(LZ_BUILD_PSDK_APP)
    if(NOT PSDK_ROOT)
        message(FATAL_ERROR "需要指定 -DPSDK_ROOT=<Payload-SDK 根目录>")
    endif()

    if(LZ_TARGET_ARCH STREQUAL "aarch64")
        set(LZ_TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
        set(LZ_ARCH_DEFINE PLATFORM_ARCH_aarch64=1)
    elseif(LZ_TARGET_ARCH STREQUAL "x86_64")
        set(LZ_TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
        set(LZ_ARCH_DEFINE PLATFORM_ARCH_x86_64=1)
    else()
        message(FATAL_ERROR "不支持的 LZ_TARGET_ARCH: ${LZ_TARGET_ARCH}（可选 aarch64 | x86_64）")
    endif()

    set(LZ_PSDK_LIB ${PSDK_ROOT}/psdk_lib/lib/${LZ_TOOLCHAIN_NAME}/libpayloadsdk.a)
    if(NOT EXISTS ${LZ_PSDK_LIB})
        message(FATAL_ERROR "找不到 PSDK 静态库: ${LZ_PSDK_LIB}")
    endif()

    # 复用官方 manifold3 样例的平台适配层（hal + osal），不重复移植
    set(LZ_M3_DIR ${PSDK_ROOT}/samples/sample_c/platform/linux/manifold3)
    set(LZ_COMMON_DIR ${PSDK_ROOT}/samples/sample_c/platform/linux/common)

    # 凭据：构建期从 lz_credentials.ini 生成 dji_sdk_app_info.h 到构建目录，
    # 并遮蔽官方样例里的同名文件 —— 明文密钥因此不进源码树。
    # 机制与理由详见 cmake/gen_app_info.cmake 与 CLAUDE.md。
    set(LZ_CREDENTIALS "${CMAKE_CURRENT_SOURCE_DIR}/lz_credentials.ini"
        CACHE FILEPATH "机载应用凭据文件（不入版本库）")
    set(LZ_APP_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

    execute_process(
        COMMAND ${CMAKE_COMMAND}
                -DLZ_CREDENTIALS=${LZ_CREDENTIALS}
                -DLZ_APP_INFO_OUT=${LZ_APP_INFO_DIR}/dji_sdk_app_info.h
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_app_info.cmake
        RESULT_VARIABLE LZ_GEN_RESULT
    )
    if(NOT LZ_GEN_RESULT EQUAL 0)
        message(FATAL_ERROR "生成 dji_sdk_app_info.h 失败，见上方输出")
    endif()

    # 平台适配层（官方 manifold3 hal/osal），两个可执行文件共用
    set(LZ_PLATFORM_SRC
        app/platform/lz_platform.c
        app/platform/lz_user_info.c
        ${LZ_M3_DIR}/hal/hal_usb_bulk.c
        ${LZ_COMMON_DIR}/osal/osal.c
        ${LZ_COMMON_DIR}/osal/osal_fs.c
        ${LZ_COMMON_DIR}/osal/osal_socket.c
    )

    add_executable(lz_app
        app/main.c
        app/lz_vision_source.c
        app/lz_mission.c
        app/lz_widget.c
        app/lz_pole_source.c
        src/lz_bridge_psdk.c
        ${LZ_PLATFORM_SRC}
    )

    # 激光测距探针 —— 独立可执行文件，用来查 M4T 哪个挂载位置有激光测距。
    # 与 lz_app 分开的理由：它是**一次性的诊断工具**，不该混进主应用的
    # 启动路径；且它需要飞机在线才有意义，主应用不该因此改变行为。
    add_executable(lz_rangefinder_probe
        app/lz_rangefinder_probe.c
        ${LZ_PLATFORM_SRC}
    )

    foreach(lz_target lz_app lz_rangefinder_probe)
        target_include_directories(${lz_target} PRIVATE
            ${LZ_APP_INFO_DIR}
            include
            app
            ${LZ_M3_DIR}/application
            ${LZ_M3_DIR}/hal
            ${LZ_COMMON_DIR}
            ${PSDK_ROOT}/psdk_lib/include
        )
        target_link_libraries(${lz_target} PRIVATE lz_core lz_vision ${LZ_PSDK_LIB} pthread m dl)
        target_compile_definitions(${lz_target} PRIVATE
            _GNU_SOURCE
            SYSTEM_ARCH_LINUX
            ${LZ_ARCH_DEFINE}
            CONFIG_HARDWARE_CONNECTION=DJI_USE_ONLY_USB_BULK_DEVICE
            $<$<BOOL:${LZ_POLE_SOURCE_LASER}>:LZ_POLE_SOURCE_LASER>
        )
        set_target_properties(${lz_target} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
        )
    endforeach()

endif()
