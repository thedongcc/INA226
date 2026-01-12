cmake_minimum_required(VERSION 3.22)

set(LIBRARY_NAME ina226)

file(GLOB_RECURSE SOURCES
        "${CMAKE_CURRENT_LIST_DIR}/src/*.c"
        "${CMAKE_CURRENT_LIST_DIR}/port/*.c"
)

add_library(${LIBRARY_NAME} STATIC ${SOURCES})
target_include_directories(${LIBRARY_NAME} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_LIST_DIR}/port
)
#target_link_libraries(${LIBRARY_NAME} PUBLIC )
