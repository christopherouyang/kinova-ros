cmake_minimum_required(VERSION 2.8.3)
project(mm_map_creator)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS visualization_msgs roscpp message_generation geometry_msgs std_msgs tf2 rviz_visual_tools pcl_ros moveit_core moveit_msgs moveit_ros_planning_interface)
find_package(octomap REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
endif()

add_service_files(FILES
  orm_calculation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  
  geometry_msgs
)

catkin_package(
  DEPENDS OpenMP
  INCLUDE_DIRS include
  LIBRARIES
    mm_sampling
    mm_discretization
    mm_kinematics
    mm_inverse_kinematics
    mm_base_placement
    mm_display
    mm_motion_planner
  CATKIN_DEPENDS message_runtime 
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

add_library(mm_collision_check include/mm_map_creator/mm_collision_check.cpp)
target_link_libraries(mm_collision_check ${catkin_LIBRARIES})

add_library(mm_discretization include/mm_map_creator/mm_discretization.cpp)
target_link_libraries(mm_discretization  ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES})

add_library(mm_sampling include/mm_map_creator/mm_sampling.cpp)
target_link_libraries(mm_sampling  ${catkin_LIBRARIES} mm_collision_check mm_kinematics mm_inverse_kinematics)

add_library(mm_display include/mm_map_creator/mm_display.cpp)
target_link_libraries(mm_display ${catkin_LIBRARIES} mm_discretization)

add_library(mm_inverse_kinematics include/mm_map_creator/mm_inverse_kinematics.cpp)
target_link_libraries(mm_inverse_kinematics ${catkin_LIBRARIES})

add_library(mm_kinematics include/mm_map_creator/mm_kinematics.cpp)
target_link_libraries(mm_kinematics ${catkin_LIBRARIES})

add_library(mm_motion_planner include/mm_map_creator/mm_motion_planner.cpp)
target_link_libraries(mm_motion_planner ${catkin_LIBRARIES})

add_library(mm_base_placement include/mm_map_creator/mm_base_placement.cpp)
target_compile_options(mm_base_placement PRIVATE ${OpenMP_FLAGS})
target_link_libraries(mm_base_placement mm_motion_planner ${catkin_LIBRARIES} ${OpenMP_LIBS})

# create rm
add_executable(mm_create_rm src/mm_create_rm.cpp)
target_link_libraries(mm_create_rm mm_sampling ${catkin_LIBRARIES})

# rm & irm down sample
add_executable(mm_down_spl src/mm_down_spl.cpp)
target_link_libraries(mm_down_spl mm_sampling ${catkin_LIBRARIES})

# create irm
add_executable(mm_create_irm src/mm_create_irm.cpp)
target_link_libraries(mm_create_irm ${catkin_LIBRARIES})

# rm & irm display
add_executable(mm_display_node src/mm_display_node.cpp)
target_link_libraries(mm_display_node ${catkin_LIBRARIES} mm_display)

# irm display
add_executable(mm_irm_display src/mm_irm_display.cpp)
target_link_libraries(mm_irm_display ${catkin_LIBRARIES} mm_display)

# orm display
add_executable(orm_display src/orm_display.cpp)
target_link_libraries(orm_display ${catkin_LIBRARIES} mm_display mm_kinematics)

# orm server
add_executable(orm_server src/orm_server.cpp)
target_link_libraries(orm_server ${catkin_LIBRARIES} mm_base_placement)