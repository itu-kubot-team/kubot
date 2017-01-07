cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_package)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(

)

find_package(PCL 1.2 REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz_cfg DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(pointcloud_subs src/pointcloud_subs.cpp)
target_link_libraries(pointcloud_subs ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_subs subscriber_cpp ${catkin_EXPORTED_TARGETS})


