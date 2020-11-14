cmake_minimum_required(VERSION 2.8.3)
project(commbot)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    nav_msgs
    geometry_msgs
    message_generation
    dynamic_reconfigure
    tf2
    tf2_ros
    tf2_geometry_msgs
)
find_package(Boost REQUIRED)

generate_dynamic_reconfigure_options(cfg/Commbot.cfg)

add_message_files(FILES
   LightStates.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   commbot
)

catkin_package(CATKIN_DEPENDS message_runtime)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_executable(commbot nodes/commbot.cpp)
add_dependencies(commbot commbot_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(commbot ${catkin_LIBRARIES})

add_executable(supervisor nodes/supervisor.cpp)
add_dependencies(supervisor ${PROJECT_NAME}_gencfg)
target_link_libraries(supervisor ${catkin_LIBRARIES})

add_executable(lights_visualizer nodes/lights_visualizer.cpp)
add_dependencies(lights_visualizer ${PROJECT_NAME}_gencfg)
target_link_libraries(lights_visualizer ${catkin_LIBRARIES})
