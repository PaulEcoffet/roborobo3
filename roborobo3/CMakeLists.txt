cmake_minimum_required(VERSION 3.5)
project(roborobo3)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-Wall -O3")

file(
        GLOB_RECURSE
        SOURCE_FILES
        src/*
        prj/*
)

include_directories(include)
include_directories(include/contrib)
include_directories(include/core)
include_directories(include/ext)
include_directories(prj)


set(Boost_USE_STATIC_LIBS ON)
if (UNIX AND NOT APPLE)
    message("it's unix")
    set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")  # use static libstdc++ which is not buggy
endif()
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")


# FindSDL2.cmake IMPORT
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Add git info as define

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(Git)


# the commit's SHA1, and whether the building workspace was dirty or not
execute_process(COMMAND
        "${GIT_EXECUTABLE}" describe --match=NeVeRmAtCh --always --abbrev=40 --dirty
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_SHA1
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the date of the commit
execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%ad --date=local
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_DATE
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# the subject of the commit
execute_process(COMMAND
        "${GIT_EXECUTABLE}" log -1 --format=%s
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        OUTPUT_VARIABLE GIT_COMMIT_SUBJECT
        ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

# generate version.cc
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/version.cpp.in" "${CMAKE_CURRENT_BINARY_DIR}/version.cpp" @ONLY)

list(APPEND SOURCE_FILES "${CMAKE_CURRENT_BINARY_DIR}/version.cpp" include/version.h)


find_package(SDL2 COMPONENTS main image REQUIRED)
#find_package(OpenCV)
if (UNIX AND NOT APPLE)
    SET(CMAKE_FIND_LIBRARY_SUFFIXES .a)
endif()
find_package(Eigen3 REQUIRED)
#find_package(TBB REQUIRED)
set (Boost_USE_STATIC_LIBS ON) # enable dynamic linking
set (Boost_USE_MULTITHREAD OFF)  # enable multithreading
find_package(Boost COMPONENTS filesystem system REQUIRED)
IF (Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
endif()

if (SDL2_FOUND)
    include_directories(${SDL2_INCLUDE_DIR})
endif()

message(${SDL2_LIBRARIES})

Set(ZLIB_DIR /usr/local/opt/zlib/)

find_package( ZLIB REQUIRED PATH ZLIB_DIR )
find_library( GZSTREAM libgzstream.a)

#include_directories(${OpenCV_INCLUDE_DIRS})
#link_directories(${OpenCV_LIB_DIR})


include_directories(${SDL2_INCLUDE_DIRS} )

add_executable(roborobo ${SOURCE_FILES} prj/Lion/src/ScoreLogger.cpp prj/Lion/include/ScoreLogger.h)


target_link_libraries(roborobo
        ${SDL2_LIBRARIES}
        ${SDL2_IMAGE_LIBRARIES}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        Eigen3::Eigen
        ${GZSTREAM}
        ${ZLIB_LIBRARIES}
        #${OpenCV_LIBS}#
        # tbb
        )
