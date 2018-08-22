cmake_minimum_required(VERSION 3.0)
project(gliders)

file(GLOB_RECURSE gliders_srcs src/glider.cpp)
add_executable(gliders ${gliders_srcs})
target_compile_options(gliders PUBLIC -std=c++14 -Wall -pedantic -Wextra -O3)

find_package(SFML 2.5 COMPONENTS graphics REQUIRED)
target_link_libraries(gliders sfml-graphics)
