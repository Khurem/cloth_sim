SET(pwd ${CMAKE_CURRENT_LIST_DIR})

SET(src "")
AUX_SOURCE_DIRECTORY(${pwd} src)
add_executable(cloth ${src})
message(STATUS "cloth added ${src}")

target_link_libraries(cloth ${stdgl_libraries})
FIND_PACKAGE(JPEG REQUIRED)
TARGET_LINK_LIBRARIES(cloth ${JPEG_LIBRARIES})
TARGET_LINK_LIBRARIES(cloth pmdreader)
