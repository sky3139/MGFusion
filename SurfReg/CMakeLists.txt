cmake_minimum_required (VERSION 3.0.2 FATAL_ERROR)
project(SurfReg)

set (CMAKE_BUILD_TYPE Release)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W -Wall")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# set(EXECUTABLE_OUTPUT_PATH ../bin)



find_package(PCL 1.7 REQUIRED)
find_package(Boost COMPONENTS serialization filesystem system thread REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()

set(CMAKE_CXX_FLAGS "-O3 -msse2 -msse3")

add_executable (surfReg SurfReg.cpp)
target_link_libraries (surfReg  ${Boost_LIBRARIES} ${PCL_LIBRARIES} -lpthread)

add_executable (hausdorff hausdorff.cc)
target_link_libraries (hausdorff  ${Boost_LIBRARIES} ${PCL_LIBRARIES} -lpthread)






find_package(Qt5Widgets)
include_directories (${PCL_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS} ${Qt5Widgets_DEFINITIONS})
qt5_generate_moc(main.cpp main.moc)
add_executable (pcd_ply_converter src/main.cpp)
target_link_libraries (pcd_ply_converter ${PCL_LIBRARIES} Qt5::Widgets)

find_package(OpenCV  REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_subdirectory(emd)

