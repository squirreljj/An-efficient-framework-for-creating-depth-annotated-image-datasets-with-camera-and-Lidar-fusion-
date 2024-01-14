project(preprocess)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)
find_package(Ceres REQUIRED)
find_library(YAML_CPP_LIB yaml-cpp)
message(STATUS "YAML_CPP_LIB=${YAML_CPP_LIB}")
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS} include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(preprocess ./src/preprocess.cpp ./src/motion_compensation.cpp)
target_link_libraries(preprocess ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(pose_transform ./src/pose_transform.cpp)
target_link_libraries(pose_transform ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)


