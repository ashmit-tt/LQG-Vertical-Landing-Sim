cmake_minimum_required(VERSION 3.10)

# 2. Name your project
project(Practice)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# where compilers should look for the header file
include_directories(include)
add_executable(Practice 
        src/main.cpp
        src/controller.cpp
        src/noise.cpp
        src/kalman.cpp
        src/simulation.cpp
        src/dynamics.cpp
        )

# Turn up warning levels for the compiler
if(MSVC)
    # If using Microsoft Visual Studio Compiler
    target_compile_options(Practice PRIVATE /W4 /WX)
else()
    # If using GCC or Clang, currently using 
    target_compile_options(Practice PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()