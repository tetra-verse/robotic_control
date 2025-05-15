get_filename_component(TARGET_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

set(PRIVATE_SOURCES 
    robotstudio.h
    robotstudio.cpp
    robotxcoredev.h
    robotxcoredev.cpp
    robotcontroller.h
    robotcontroller.cpp
)

add_library(${TARGET_NAME} STATIC ${PRIVATE_SOURCES})
target_link_libraries(${TARGET_NAME} PRIVATE log CoreSDK eigen)
