cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

project(convsim)

if(NOT DEFINED SYSTEMC_HOME)
    if(DEFINED ENV{SYSTEMC_HOME})
        set(SYSTEMC_HOME ENV{SYSTEMC_HOME})
    else()
        message(FATAL_ERROR "You need to set the SYSTEMC_HOME environment variable!")
    endif()
endif()

include_directories(${SYSTEMC_HOME}/include)
link_directories(${SYSTEMC_HOME}/lib-linux64)
add_definitions(-DSC_DISABLE_API_VERSION_CHECK -DSC_INCLUDE_DYNAMIC_PROCESSES)

FILE(GLOB SRCFILES *.cpp)
FILE(GLOB HDRFILES *.h)

add_executable(${PROJECT_NAME} ${SRCFILES} ${HDRFILES})
target_link_libraries(${PROJECT_NAME} systemc)
