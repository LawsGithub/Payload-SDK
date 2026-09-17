# ===========================================================================
# 从 wt_credentials.ini 生成 dji_sdk_app_info.h
#
# 为什么单独写一个脚本而不是用 configure_file：
#   凭据串里会出现 '+' '/' '=' 以及中文，若走 configure_file 的 @VAR@ 展开，
#   任何一处被 CMake 当成变量语法都会静默改写内容 —— 而这类错误的后果是
#   "连不上飞机"，排查成本极高。这里逐行按 key=value 取值后原样拼接，
#   全程不做变量展开，且拒绝含引号或反斜杠的值，不给转义留出错空间。
#
# 用法：
#   cmake -DWT_CREDENTIALS=<ini> -DWT_APP_INFO_OUT=<输出头文件> \
#         -P cmake/gen_app_info.cmake
#
# 凭据文件不存在时生成占位符版本并给出 WARNING：源码自带的头文件本来也是
# 占位符，行为一致；程序启动时会明确拒绝并提示，比含糊的"授权失败"好定位。
# ===========================================================================

set(WT_APP_NAME          "your_app_name")
set(WT_APP_ID            "your_app_id")
set(WT_APP_KEY           "your_app_key")
set(WT_APP_LICENSE       "your_app_license")
set(WT_DEVELOPER_ACCOUNT "your_developer_account")
set(WT_BAUD_RATE         "460800")

# 标记每个必需键是否出现过 —— 文件存在却漏填时应当立刻失败，
# 而不是安静地退回占位符（那就成了"配了却没生效"）
set(WT_SEEN_NAME     FALSE)
set(WT_SEEN_ID       FALSE)
set(WT_SEEN_KEY      FALSE)
set(WT_SEEN_LICENSE  FALSE)
set(WT_SEEN_ACCOUNT  FALSE)

if(EXISTS "${WT_CREDENTIALS}")
    file(STRINGS "${WT_CREDENTIALS}" WT_LINES ENCODING UTF-8)

    foreach(wt_line IN LISTS WT_LINES)
        if(wt_line MATCHES "^[ \t]*#" OR wt_line MATCHES "^[ \t]*$")
            continue()
        endif()

        if(NOT wt_line MATCHES "^([A-Za-z_][A-Za-z0-9_]*)=(.*)$")
            message(FATAL_ERROR
                "凭据文件存在无法解析的行：\n    ${wt_line}\n"
                "格式应为 key=value，# 开头为注释。文件：${WT_CREDENTIALS}")
        endif()

        set(wt_key "${CMAKE_MATCH_1}")
        set(wt_val "${CMAKE_MATCH_2}")

        # C 字符串字面量里无法安全容纳这两个字符；与其做转义，不如直接拒收
        if(wt_val MATCHES "[\"\\\\]")
            message(FATAL_ERROR
                "凭据项 ${wt_key} 的值含有引号或反斜杠，无法安全写入 C 头文件。\n"
                "请检查 ${WT_CREDENTIALS}")
        endif()

        if(wt_key STREQUAL "app_name")
            set(WT_APP_NAME "${wt_val}")
            set(WT_SEEN_NAME TRUE)
        elseif(wt_key STREQUAL "app_id")
            set(WT_APP_ID "${wt_val}")
            set(WT_SEEN_ID TRUE)
        elseif(wt_key STREQUAL "app_key")
            set(WT_APP_KEY "${wt_val}")
            set(WT_SEEN_KEY TRUE)
        elseif(wt_key STREQUAL "app_license")
            set(WT_APP_LICENSE "${wt_val}")
            set(WT_SEEN_LICENSE TRUE)
        elseif(wt_key STREQUAL "developer_account")
            set(WT_DEVELOPER_ACCOUNT "${wt_val}")
            set(WT_SEEN_ACCOUNT TRUE)
        elseif(wt_key STREQUAL "baud_rate")
            set(WT_BAUD_RATE "${wt_val}")
        else()
            # 与配置解析同一套主张：未知键名是错误，不是可以忽略的注释
            message(FATAL_ERROR
                "凭据文件存在未知键名：${wt_key}\n文件：${WT_CREDENTIALS}")
        endif()
    endforeach()

    foreach(wt_pair
            "app_name;${WT_SEEN_NAME}"
            "app_id;${WT_SEEN_ID}"
            "app_key;${WT_SEEN_KEY}"
            "app_license;${WT_SEEN_LICENSE}"
            "developer_account;${WT_SEEN_ACCOUNT}")
        list(GET wt_pair 0 wt_k)
        list(GET wt_pair 1 wt_seen)
        if(NOT wt_seen)
            set(WT_MISSING "${WT_MISSING} ${wt_k}")
        endif()
    endforeach()

    if(WT_MISSING)
        message(FATAL_ERROR
            "凭据文件缺少必需项:${WT_MISSING}\n文件：${WT_CREDENTIALS}")
    endif()

    message(STATUS "已从 ${WT_CREDENTIALS} 载入机载应用凭据（App ID ${WT_APP_ID}）")
else()
    message(WARNING
        "未找到凭据文件 ${WT_CREDENTIALS}，将生成占位符版 dji_sdk_app_info.h。\n"
        "程序能编出来，但启动时会拒绝运行。创建方式见 wt_inspection/README.md。")
endif()

get_filename_component(WT_OUT_DIR "${WT_APP_INFO_OUT}" DIRECTORY)
file(MAKE_DIRECTORY "${WT_OUT_DIR}")

file(WRITE "${WT_APP_INFO_OUT}" "/**
 ********************************************************************
 * @file    dji_sdk_app_info.h
 * @brief   机载应用身份信息 —— 构建期生成，请勿编辑
 *
 * 本文件由 wt_inspection/cmake/gen_app_info.cmake 生成，来源为
 * wt_inspection/wt_credentials.ini（该文件不入版本库）。
 * 手工修改会在下次配置时被覆盖。
 *
 * 之所以走生成而不是直接改源码里的同名头文件：凭据是明文，
 * 一旦提交就永久留在 git 历史里。生成的文件落在构建目录，
 * 与源码树和版本库都隔离。
 *********************************************************************
 */

#ifndef DJI_SDK_APP_INFO_H
#define DJI_SDK_APP_INFO_H

#ifdef __cplusplus
extern \"C\" {
#endif

/* Exported constants --------------------------------------------------------*/
#define USER_APP_NAME               \"${WT_APP_NAME}\"
#define USER_APP_ID                 \"${WT_APP_ID}\"
#define USER_APP_KEY                \"${WT_APP_KEY}\"
#define USER_APP_LICENSE            \"${WT_APP_LICENSE}\"
#define USER_DEVELOPER_ACCOUNT      \"${WT_DEVELOPER_ACCOUNT}\"
#define USER_BAUD_RATE              \"${WT_BAUD_RATE}\"

#ifdef __cplusplus
}
#endif

#endif // DJI_SDK_APP_INFO_H
")