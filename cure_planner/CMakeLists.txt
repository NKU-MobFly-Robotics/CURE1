cmake_minimum_required(VERSION 3.0.2)

project(cure_planner)

#for Robot class
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CAMKE_CXX_FLAGS_RELEASE "-03 -wall -g -pthread")
#add_compile_options(-std=c++11)
##add_library()
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  cv_bridge 
  image_transport 
  roscpp 
  sensor_msgs 
  std_msgs 
  tf 
  actionlib)
# OpenCV is required for merging without initial positions
find_package(OpenCV 4 REQUIRED)
 if(OpenCV_VERSION VERSION_LESS "4.0")
  message(FATAL_ERROR "This package needs OpenCV >= 4.0")
 endif()


include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} include) 


add_message_files(FILES
  PointArray.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime)



add_executable(frontiers
  src/frontiers.cpp
)
add_dependencies(frontiers ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontiers
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(get_voronoi_centroid
  src/get_voronoi_centroid.cpp
)
add_dependencies(get_voronoi_centroid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_voronoi_centroid
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(robot_position
  src/robot_position.cpp
)
add_dependencies(robot_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_position
  ${catkin_LIBRARIES}
)

add_executable(target_assigner
  src/target_assigner.cpp
)
add_dependencies(target_assigner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(target_assigner
  ${catkin_LIBRARIES}
)

add_executable(merge_map
 src/merge_map.cpp
)
add_dependencies(merge_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(merge_map
 ${catkin_LIBRARIES}
)

