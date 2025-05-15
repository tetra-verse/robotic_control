
set(PRIVATE_SOURCES 
    blockingqueue.h
)

add_library(utils STATIC ${PRIVATE_SOURCES})

target_link_libraries(utils
  PRIVATE )
