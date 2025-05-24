cmake_minimum_required(VERSION 3.10)

# 定义 krnx 库目标
set(TARGET_NAME krnx)

# 设置目录变量
set(KRNX_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(KRNX_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(KRNX_BIN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin)

# 添加导入的共享库
add_library(${TARGET_NAME} SHARED IMPORTED GLOBAL)

# 设置目标属性
if(WIN32)
    set_target_properties(${TARGET_NAME} PROPERTIES
        IMPORTED_LOCATION "${KRNX_LIB_DIR}/krnx.dll"
        IMPORTED_IMPLIB "${KRNX_LIB_DIR}/krnx.lib"
    )
    set(KRNX_LINK_LIB krnx)
else()
    # 为其他平台添加支持（如需要）
    message(WARNING "krnx 当前仅支持 Windows 平台")
    set(KRNX_LINK_LIB krnx)
endif()

# 设置接口包含目录
target_include_directories(${TARGET_NAME} 
                          INTERFACE ${KRNX_INCLUDE_DIR})

# 设置接口链接目录和库
target_link_directories(${TARGET_NAME} 
                       INTERFACE ${KRNX_LIB_DIR})
target_link_libraries(${TARGET_NAME} 
                     INTERFACE ${KRNX_LINK_LIB})

# 创建自定义命令来复制运行时文件
add_custom_target(
    copy_krnx_runtime_files ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory 
            ${KRNX_BIN_DIR} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    COMMENT "正在复制 KRNX 运行时文件到输出目录"
)

# 添加依赖关系
add_dependencies(${TARGET_NAME} copy_krnx_runtime_files)

# 输出配置信息
message(STATUS "KRNX 库已配置：")
message(STATUS "  - 包含目录: ${KRNX_INCLUDE_DIR}")
message(STATUS "  - 库目录: ${KRNX_LIB_DIR}")
message(STATUS "  - 运行时文件目录: ${KRNX_BIN_DIR}")
message(STATUS "  - 输出目录: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

# 安装规则
include(GNUInstallDirs)

# 安装头文件
install(
    DIRECTORY ${KRNX_INCLUDE_DIR}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/krnx
    FILES_MATCHING PATTERN "*.h"
    PATTERN "*.hpp"
)

# 安装库文件
if(WIN32)
    install(
        FILES 
            "${KRNX_LIB_DIR}/krnx.lib"
            "${KRNX_LIB_DIR}/krnx.dll"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
else()
    # 其他平台的库文件安装（如需要）
    message(STATUS "非 Windows 平台的库文件安装未配置")
endif()

# 安装运行时文件
install(
    DIRECTORY ${KRNX_BIN_DIR}/
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)


