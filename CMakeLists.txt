cmake_minimum_required(VERSION 3.0.2)
project(object_tracking_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_tracking_ros
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(hungarian_algorithm src/hungarian_algorithm.cpp)
add_library(kalman_filter src/kalman_filter.cpp)
add_library(track src/track.cpp)
add_library(track_manager src/track_manager.cpp)
add_library(multi_object_tracker src/multi_object_tracker.cpp)

add_dependencies(hungarian_algorithm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(kalman_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(track ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(track_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(multi_object_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hungarian_algorithm ${catkin_LIBRARIES})
target_link_libraries(kalman_filter ${catkin_LIBRARIES} Eigen3::Eigen)
target_link_libraries(track ${catkin_LIBRARIES} kalman_filter)
target_link_libraries(track_manager ${catkin_LIBRARIES} track hungarian_algorithm)
target_link_libraries(multi_object_tracker ${catkin_LIBRARIES} track_manager ${OpenCV_LIBRARIES})

add_executable(object_tracking_ros_node src/main.cpp)
target_link_libraries(object_tracking_ros_node multi_object_tracker)
add_dependencies(object_tracking_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})