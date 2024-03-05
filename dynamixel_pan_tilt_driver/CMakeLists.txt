cmake_minimum_required(VERSION 3.1)
project(dynamixel_pan_tilt_driver VERSION 0.1)

add_compile_options(-std=c++14)

set(${PROJECT_NAME}_CATKIN_DEPS
    roscpp
    dynamixel_sdk
    dynamixel_pan_tilt_msgs
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
    INCLUDE_DIRS include config launch
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
    src/dynamixel_pan_tilt_driver/indirect_sync_read.cpp
    src/dynamixel_pan_tilt_driver/indirect_sync_write.cpp
    src/dynamixel_pan_tilt_driver/pan_tilt_controller.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


add_executable(dynamixel_pan_tilt_node src/dynamixel_pan_tilt_node.cpp)
target_link_libraries(dynamixel_pan_tilt_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# add_executable(test_t src/test_t.cpp)
# target_link_libraries(test_t ${PROJECT_NAME} ${catkin_LIBRARIES})