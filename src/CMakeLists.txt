


# 可执行程序
set(TARGET_NAME ${PROJECT_NAME})
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 当前功能路径
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#add_subdirectory(utils)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package (Threads REQUIRED)


find_package(QT NAMES Qt6 Qt5 COMPONENTS Core REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core REQUIRED)

add_executable(
  ${TARGET_NAME}
  main.cpp
 )

target_link_libraries(
  ${TARGET_NAME}
    log
    dev
    app
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    Qt${QT_VERSION_MAJOR}::Core
    Threads::Threads)

set_target_properties(${TARGET_NAME} PROPERTIES DEBUG_POSTFIX
                                                "${CMAKE_DEBUG_POSTFIX}")

add_subdirectory(log)
add_subdirectory(dev)
add_subdirectory(app)

include(GNUInstallDirs)
install(TARGETS ${TARGET_NAME} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
