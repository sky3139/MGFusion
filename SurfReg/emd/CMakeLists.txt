cmake_minimum_required(VERSION 2.8.3)
project(emd_poses)

find_package (PCL REQUIRED COMPONENTS io common visualization features) 
find_package(OpenCV REQUIRED)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_executable(emd_poses ${PROJECT_SOURCE_DIR}/emd_poses.cpp)
target_link_libraries(emd_poses   ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

