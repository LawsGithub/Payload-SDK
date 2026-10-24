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
    src/wt_blade_id.c
    src/wt_projection.c
    src/wt_ai_solve.c
    src/wt_rotor_fit.c
    src/wt_aim.c
    src/wt_survey_logic.c
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

    foreach(wt_test wt_test_plan wt_test_config wt_test_bridge wt_test_blade_id wt_test_projection wt_test_ai_solve wt_test_rotor_fit wt_test_aim wt_test_telemetry wt_test_survey_logic)
        add_executable(${wt_test} tests/${wt_test}.c)
        target_include_directories(${wt_test} PRIVATE tests)
        target_link_libraries(${wt_test} PRIVATE wt_core)
        add_test(NAME ${wt_test} COMMAND ${wt_test})
        # 部分用例要写临时配置文件，给它们一个确定可写的工作目录
        set_tests_properties(${wt_test} PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    endforeach()

    # wt_test_ai_bridge 需要 app/ 的 include 路径（WtTrtPose 的类型定义在
    # 那里）且要多编一个源文件，不满足上面 foreach 的统一假设，单独写。
    # 只依赖 wt_trt_pose.h 的类型声明，不链接 TensorRT，因此可在桌面编译测试。
    add_executable(wt_test_ai_bridge tests/wt_test_ai_bridge.c app/wt_ai_bridge.c)
    target_include_directories(wt_test_ai_bridge PRIVATE tests app)
    target_link_libraries(wt_test_ai_bridge PRIVATE wt_core)
    add_test(NAME wt_test_ai_bridge COMMAND wt_test_ai_bridge)
    set_tests_properties(wt_test_ai_bridge PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
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

# TensorRT 推理与激光采集链路。只能在妙算3 上开（见下方 if 块的说明）。
option(WT_BUILD_TRT "编译 TensorRT 推理与激光采集链路（仅设备上可编译）" OFF)

# 妙算3 出厂镜像的实测路径（2026-10-22 核实）：nvcc 与 trtexec 不在 PATH 里，
# CUDA 在 /usr/local/cuda-11.4，TensorRT 头文件在系统 include 路径下。
set(WT_CUDA_INCLUDE_DIR "/usr/local/cuda-11.4/include"
    CACHE PATH "CUDA 头文件目录（设备上）")
set(WT_CUDA_LIB_DIR "/usr/local/cuda-11.4/lib64"
    CACHE PATH "CUDA 库目录（设备上）")
set(WT_TRT_INCLUDE_DIR "/usr/include/aarch64-linux-gnu"
    CACHE PATH "TensorRT 头文件目录（设备上）")
set(WT_TRT_LIB_DIR "/usr/lib/aarch64-linux-gnu"
    CACHE PATH "TensorRT 库目录（设备上）")

if(WT_BUILD_PSDK_APP)
    if(NOT PSDK_ROOT)
        message(FATAL_ERROR "需要指定 -DPSDK_ROOT=<Payload-SDK 根目录>")
    endif()

    # -----------------------------------------------------------------------
    # 机载应用凭据：构建期生成 dji_sdk_app_info.h
    #
    # 凭据是明文密钥，不能进源码树（一旦提交就永久留在 git 历史里）。
    # 这里从 wt_credentials.ini 读出，生成头文件到构建目录，并把该目录
    # 放在头文件搜索路径最前面 —— 从而遮蔽官方样例里那份同名头文件。
    #
    # 生成的文件名与官方样例**故意保持一致**：这样 PSDK 分发包里任何
    # #include "dji_sdk_app_info.h" 都无需改动（包括 manifold3 自带样例）。
    # 不采用「改源码文件」的做法，是因为 CMake 无条件把
    # ${WT_M3_DIR}/application 加进搜索路径，源码树里那份同名文件会形成
    # 第二份真值来源，日后极易被误编辑。
    # -----------------------------------------------------------------------
    set(WT_CREDENTIALS "${CMAKE_CURRENT_SOURCE_DIR}/wt_credentials.ini"
        CACHE FILEPATH "机载应用凭据文件（不入版本库）")
    set(WT_APP_INFO_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

    execute_process(
        COMMAND ${CMAKE_COMMAND}
                -DWT_CREDENTIALS=${WT_CREDENTIALS}
                -DWT_APP_INFO_OUT=${WT_APP_INFO_DIR}/dji_sdk_app_info.h
                -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_app_info.cmake
        RESULT_VARIABLE WT_GEN_RESULT
    )
    if(NOT WT_GEN_RESULT EQUAL 0)
        message(FATAL_ERROR "生成 dji_sdk_app_info.h 失败，见上方输出")
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
        app/wt_ai_bridge.c
        app/wt_ai_overlay.c
        app/wt_ai_stream_probe.c
        app/wt_frame_pump.c
        app/wt_laser.c
        app/wt_widget.c
        src/wt_telemetry_psdk.c
        src/wt_runner_psdk.c
        ${WT_M3_DIR}/hal/hal_usb_bulk.c
        ${WT_COMMON_DIR}/osal/osal.c
        ${WT_COMMON_DIR}/osal/osal_fs.c
        ${WT_COMMON_DIR}/osal/osal_socket.c
    )

    # -----------------------------------------------------------------------
    # TensorRT 推理与激光采集链路（默认关闭）
    #
    # 为什么是开关而不是直接编进去：engine 与 TensorRT 库只在妙算3 上有，
    # 而仓库级 CLAUDE.md 的硬约束 3 是「绝不交叉编译」—— 本机 glibc 2.35
    # 高于设备 2.31，交叉产物在设备上 ldd 直接报 not found。所以这部分
    # **只能在设备上开启编译**，本机构建保持可用。
    #
    #   设备上：cmake -S . -B build -DWT_BUILD_PSDK_APP=ON -DWT_BUILD_TRT=ON \
    #                 -DPSDK_ROOT=<...>
    #
    # frame_pump 与 laser 不在这个开关里 —— 它们只依赖 PSDK，不碰 TensorRT，
    # 因此可以先单独验证取流与激光链路，再接推理。
    # -----------------------------------------------------------------------
    if(WT_BUILD_TRT)
        enable_language(CXX)
        set(CMAKE_CXX_STANDARD 14)
        set(CMAKE_CXX_STANDARD_REQUIRED ON)

        target_sources(wt_inspection_app PRIVATE
            app/wt_trt_pose.cpp
            app/wt_vision.c
            app/wt_survey.c
        )

        target_include_directories(wt_inspection_app PRIVATE
            ${WT_CUDA_INCLUDE_DIR}
            ${WT_TRT_INCLUDE_DIR}
        )

        # 用 find_library 取全路径，不用 target_link_directories ——
        # 后者要 CMake 3.13+，而本工程声明的最低版本是 3.10。
        # 顺带好处：库缺失在**配置阶段**就报错，而不是等到链接才失败。
        find_library(WT_NVINFER_LIB nvinfer
                     HINTS ${WT_TRT_LIB_DIR} /usr/lib/aarch64-linux-gnu)
        find_library(WT_CUDART_LIB cudart
                     HINTS ${WT_CUDA_LIB_DIR} /usr/local/cuda/lib64)

        if(NOT WT_NVINFER_LIB)
            message(FATAL_ERROR "找不到 libnvinfer。设备上应在 /usr/lib/aarch64-linux-gnu；"
                                "可用 -DWT_TRT_LIB_DIR=<路径> 指定")
        endif()
        if(NOT WT_CUDART_LIB)
            message(FATAL_ERROR "找不到 libcudart。可用 -DWT_CUDA_LIB_DIR=<路径> 指定")
        endif()

        target_link_libraries(wt_inspection_app PRIVATE
            ${WT_NVINFER_LIB}
            ${WT_CUDART_LIB}
            stdc++
        )

        target_compile_definitions(wt_inspection_app PRIVATE WT_WITH_TRT=1)

        message(STATUS "已启用 TensorRT 链路：CUDA=${WT_CUDA_INCLUDE_DIR} TRT=${WT_TRT_INCLUDE_DIR}")
    endif()

    target_include_directories(wt_inspection_app PRIVATE
        ${WT_APP_INFO_DIR}
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