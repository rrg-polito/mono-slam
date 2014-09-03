cmake_minimum_required (VERSION 2.6)
project(mono-slam)

set (Tutorial_VERSION_MAJOR b2)
set (Tutorial_VERSION_MINOR 0)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../cmake)

find_package( OpenCV REQUIRED )

find_package(LibConfig++ REQUIRED)
include_directories(${LIBCONFIG++_INCLUDE_DIR})



include_directories ("${PROJECT_SOURCE_DIR}/lib")

add_library(monoslam
    lib/ConfigVSLAM.cpp
    lib/Patch.cpp
    lib/camModel.cpp
    lib/libblur.cpp
    lib/utils.cpp
    lib/vslamRansac.cpp
    )

target_link_libraries (monoslam ${OpenCV_LIBS} ${LIBCONFIG++_LIBRARY})

add_executable(test
    examples/test.cpp
    )

target_link_libraries (test monoslam ${OpenCV_LIBS} ${LIBCONFIG++_LIBRARY})
