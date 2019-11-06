cmake_minimum_required(VERSION 2.8)

project(convsim)

set(SYSTEMC_HOME "/home/vscotti/Apps/systemc-2.3.3/")

include_directories(${SYSTEMC_HOME}/include)
link_directories(${SYSTEMC_HOME}/lib-linux64)

add_executable(${PROJECT_NAME} "main.cpp")
target_link_libraries(${PROJECT_NAME} systemc)
