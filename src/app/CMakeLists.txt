
set(PRIVATE_SOURCES 
    robotapp.h
    robotapp.cpp
)

add_library(app STATIC ${PRIVATE_SOURCES})

target_link_libraries(app
  PRIVATE 
    dev
    Rokae::Rokae
    eigen)
