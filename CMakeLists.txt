cmake_minimum_required(VERSION 3.0)
project(gliders)

file(GLOB_RECURSE gliders_srcs src/glider.cpp)
add_executable(gliders ${gliders_srcs})
target_compile_options(gliders PUBLIC -std=c++14 -Wall -pedantic -Wextra -O3)

find_package(SFML 2.5 COMPONENTS graphics REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(Boost_USE_STATIC_LIBS  ON)
find_package(Boost 1.67 COMPONENTS system filesystem REQUIRED)

target_link_libraries(gliders
  sfml-graphics
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  Eigen3::Eigen
)
