cmake_minimum_required(VERSION 3.10)
project(whole_body_controller LANGUAGES CXX)

#Set environment variables
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

#cmake config directories
set(raisim_DIR ./libs/raisim/lib/cmake/raisim)
set(osqp_DIR ./libs/osqp/build)

#find packages
find_package(raisim CONFIG REQUIRED)
find_package(osqp REQUIRED)

#includes
include_directories(
    ./include 
    ./libs/eigen3/include 
    ./libs/osqp/include
    ./libs/qpoases/include 
    ./libs/matplotlib-cpp 
    /home/user/anaconda3/envs/imitation_learning/include/python3.7m
)

#executables
add_executable(${PROJECT_NAME} 
    ./tests/whole_body_controller.cpp 
    ./src/a1.cpp
    ./src/gait_generator.cpp
    ./src/stance_controller.cpp
    ./src/swing_controller.cpp
    ./src/mpc_osqp.cc
    ./src/locomotion_controller.cpp
)

#Link external libraries
target_link_libraries(${PROJECT_NAME} 
    PRIVATE osqp::osqp
    PUBLIC raisim::raisim pthread 
    ${CMAKE_SOURCE_DIR}/libs/qpoases/build/libs/libqpOASES.a 
    /home/user/anaconda3/envs/imitation_learning/lib/libpython3.7m.so
)

target_include_directories(${PROJECT_NAME} PUBLIC ./libs/raisim/include)
