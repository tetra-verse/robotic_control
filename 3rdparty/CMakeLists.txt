add_subdirectory(fmt-10.x)

set(SPDLOG_FMT_EXTERNAL ON CACHE BOOL "" FORCE)
add_subdirectory(spdlog)
add_subdirectory(json)

set(BUILD_SHARED_LIBS OFF)
