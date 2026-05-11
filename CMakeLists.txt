cmake_minimum_required(VERSION 3.16)

project(bgOpen3D LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Open3D REQUIRED)

add_executable(bgOpen3D
	src/main.cpp
	src/point_cloud_merge.cpp
	src/point_cloud_poisson.cpp
	src/point_cloud_ball_pivoting.cpp
	src/point_cloud_post_merge_processing.cpp
)
target_include_directories(bgOpen3D PRIVATE include)
target_link_libraries(bgOpen3D PRIVATE Open3D::Open3D)

add_executable(point_cloud_post_merge
	src/point_cloud_post_merge_processing.cpp
	src/point_cloud_post_merge.cpp
)
target_include_directories(point_cloud_post_merge PRIVATE include)
target_link_libraries(point_cloud_post_merge PRIVATE Open3D::Open3D)