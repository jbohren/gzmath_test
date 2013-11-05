cmake_minimum_required(VERSION 2.8.3)
project(gzmath_test)

find_package(catkin REQUIRED)

find_package(gazebo REQUIRED)

catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS})
add_executable(posediff posediff.cpp)
target_link_libraries(posediff ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

