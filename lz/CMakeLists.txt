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
    foreach(lz_test lz_test_geo lz_test_plan lz_test_bridge lz_test_kmz lz_test_wpml lz_test_validate)
        add_executable(${lz_test} tests/${lz_test}.c)
        target_include_directories(${lz_test} PRIVATE tests)
        target_link_libraries(${lz_test} PRIVATE lz_core)
        add_test(NAME ${lz_test} COMMAND ${lz_test})
        set_tests_properties(${lz_test} PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endforeach()

    # ---- 探杆位记录：单独一条，因为它要编 app/ 下的源文件 ----
    #
    # 上面 6 个只链接 lz_core（纯算法）。这个要编 `app/lz_pole_source.c`，
    # 它 include 了 `dji_logger.h` —— 用 tests/stub/ 里的**替身**头文件
    # 把日志宏换成 printf，就能上桌面。
    #
    # 替身只替换日志宏，被测的 lz_pole_source.c 原封不动编译。
    # 详见 tests/stub/dji_logger.h 与 tests/lz_test_pole.c 的文件头注释。
    add_executable(lz_test_pole
        tests/lz_test_pole.c
        app/lz_pole_source.c
        src/lz_types.c
    )
    # stub 放在前面：`#include <dji_logger.h>` 会优先命中替身
    target_include_directories(lz_test_pole PRIVATE tests/stub tests include app)
    target_link_libraries(lz_test_pole PRIVATE m)
    add_test(NAME lz_test_pole COMMAND lz_test_pole)
    set_tests_properties(lz_test_pole PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
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

# ---- 视觉层的回归测试：对 6 张**真实俯拍照片**跑 ----
#
# ⚠️ 只在 hsv 后端下有意义：stub 后端刻意不看画面，它对照片的检测结果
# 与照片内容无关，比对了也证明不了什么。
#
# 测试数据（tests/data/*.ppm + vision_golden.txt）由
# tools/gen_vision_testdata.js 从现场照片生成，约 1.5 MB，**入库** ——
# 它的价值就在于"不需要现场照片也能复现检测行为"，不入库等于没有。
if(LZ_VISION_BACKEND STREQUAL "hsv" AND LZ_BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    add_executable(lz_test_vision tests/lz_test_vision.c)
    target_include_directories(lz_test_vision PRIVATE tests)
    target_link_libraries(lz_test_vision PRIVATE lz_vision m)
    add_test(NAME lz_test_vision COMMAND lz_test_vision)
    # 数据目录用绝对路径传给测试 —— 测试的 CWD 是构建目录，
    # 而数据在源树里（tests/data）。写相对路径会在 build/ 下找不到。
    set_tests_properties(lz_test_vision PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        ENVIRONMENT "LZ_VISION_TESTDATA=${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
endif()

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

# 激光记录按钮是否可用。
#
# ⚠️ 2026-09-22 起默认 **ON** —— 依据是用户要求"两个按钮都要能用"，
# 且激光测距在 M4T 位置 1 上已实测可用（见 CLAUDE.md「激光测距」一节）。
#
# 打开它的代价仅两处（都在 main.c 里条件编译）：
#   1. 启动时多调一次 DjiCameraManager_Init()
#   2. 退出时多调一次 DjiCameraManager_DeInit()
# 相机模块**不**改变启动路径的成败 —— 初始化失败只让激光按钮不可用。
#
# 关掉它（-DLZ_POLE_SOURCE_LASER=OFF）的场景：包体要最小、
# 或现场确认不用激光（那只剩"记录飞机位"一条路）。
option(LZ_POLE_SOURCE_LASER "启用 PSDK 相机模块，使「记录激光点」按钮可用" ON)

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
        app/platform/lz_sdk_log_watch.c
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

    # 取图探针 —— 查 M4T 的图像流能不能取到、取到的是哪个镜头的画面。
    #
    # 与上面两个探针同一个理由：**把不确定性隔离在一次上机试验里**。
    # 取图的 position/source/pixFmt 现在都是猜的，而猜错的表现是
    # "花屏"或"收不到帧"而不是报错 —— 在主应用里排查这件事，
    # 要连带着任务状态机一起看，成本高得多。
    # 详见 lz/app/lz_liveview_probe.c 的文件头与 doc/VISION-GIMBAL-PITCH.md §9 #1。
    add_executable(lz_liveview_probe
        app/lz_liveview_probe.c
        app/lz_vision_source.c
        ${LZ_PLATFORM_SRC}
    )

    # 任务链路探针 —— 复现"上传成功但启动航点时崩溃"，把崩溃点夹出来。
    # 与 lz_app 分开的理由同激光探针：它是**一次性的诊断工具**，
    # 且必须把变量收干净（不建线程、不注册控件），才能分清
    # 崩溃是在 SDK 内部还是在我们的诊断代码里。
    add_executable(lz_mission_probe
        app/lz_mission_probe.c
        ${LZ_PLATFORM_SRC}
    )

    foreach(lz_target lz_app lz_rangefinder_probe lz_mission_probe lz_liveview_probe)
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
