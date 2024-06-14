cmake_minimum_required(VERSION 3.16)
project(stand_go2)

find_package(OsqpEigen REQUIRED)
find_package(mujoco REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3)

include_directories( /home/jang/unitree_mujoco/example/cpp/include 
                    /usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2 
                    ${EIGEN3_INCLUDE_DIR})

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

add_executable(stand_go2 src/communication.cpp src/main.cpp src/controller.cpp)

target_link_libraries(stand_go2 OsqpEigen::OsqpEigen mujoco::mujoco pinocchio::pinocchio glfw)

SET(CMAKE_BUILD_TYPE Release)
