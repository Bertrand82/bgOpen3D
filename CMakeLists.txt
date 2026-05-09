cmake_minimum_required(VERSION 3.16)

project(georeloc VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

# Reprend la logique du projet voisin pour utiliser une toolchain CUDA/COLMAP/OpenCV locale
# quand elle est disponible sur la machine de dev.
set(_LOCAL_CUDA_12_8_DIR "/usr/local/cuda-12.8")
if(EXISTS "${_LOCAL_CUDA_12_8_DIR}/bin/nvcc")
  set(CUDAToolkit_ROOT "${_LOCAL_CUDA_12_8_DIR}" CACHE PATH "" FORCE)
  set(CUDA_TOOLKIT_ROOT_DIR "${_LOCAL_CUDA_12_8_DIR}" CACHE PATH "" FORCE)
  set(CMAKE_CUDA_COMPILER "${_LOCAL_CUDA_12_8_DIR}/bin/nvcc" CACHE FILEPATH "" FORCE)
endif()

set(_LOCAL_COLMAP_DIR "$ENV{HOME}/.local/colmap/share/colmap")
if(EXISTS "${_LOCAL_COLMAP_DIR}")
  set(colmap_DIR "${_LOCAL_COLMAP_DIR}" CACHE PATH "" FORCE)
endif()

find_package(colmap CONFIG QUIET)
if(NOT colmap_FOUND)
  find_package(COLMAP CONFIG QUIET)
endif()

if(NOT colmap_FOUND AND NOT COLMAP_FOUND)
  message(FATAL_ERROR
    "COLMAP n'a pas ete trouve. Definissez -Dcolmap_DIR=/chemin/vers/colmap"
    " ou -DCOLMAP_DIR=/chemin/vers/COLMAP."
  )
endif()

set(COLMAP_LIB_TARGET "")
if(TARGET colmap::colmap)
  set(COLMAP_LIB_TARGET colmap::colmap)
elseif(TARGET COLMAP::colmap)
  set(COLMAP_LIB_TARGET COLMAP::colmap)
endif()

if(COLMAP_LIB_TARGET STREQUAL "")
  message(FATAL_ERROR "Le package COLMAP est present mais la cible de lien (colmap::colmap ou COLMAP::colmap) est introuvable.")
endif()

if(DEFINED CUDAToolkit_VERSION_MAJOR AND DEFINED CUDAToolkit_VERSION_MINOR)
  set(CUDA_VERSION_STRING "${CUDAToolkit_VERSION_MAJOR}.${CUDAToolkit_VERSION_MINOR}")
elseif(DEFINED CUDAToolkit_VERSION)
  string(REGEX MATCH "^[0-9]+\\.[0-9]+" _CUDA_VERSION_MAJOR_MINOR "${CUDAToolkit_VERSION}")
  if(_CUDA_VERSION_MAJOR_MINOR)
    set(CUDA_VERSION_STRING "${_CUDA_VERSION_MAJOR_MINOR}")
  endif()
endif()

set(_LOCAL_OPENCV_CUDA_DIR "/home/bertrand/workspaceCpp/opencv_build_cuda")
if(EXISTS "${_LOCAL_OPENCV_CUDA_DIR}/OpenCVConfig.cmake")
  set(OpenCV_DIR "${_LOCAL_OPENCV_CUDA_DIR}" CACHE PATH "" FORCE)
endif()

set(_LOCAL_OPEN3D_DIR "$ENV{HOME}/.local/open3d/lib/cmake/Open3D")
if(EXISTS "${_LOCAL_OPEN3D_DIR}")
  set(Open3D_DIR "${_LOCAL_OPEN3D_DIR}" CACHE PATH "" FORCE)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc features2d calib3d)
find_package(Eigen3 REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(Open3D REQUIRED)

file(GLOB_RECURSE GEORELOC_SOURCE_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cxx"
)

file(GLOB_RECURSE GEORELOC_TEST_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cxx"
)

if(GEORELOC_SOURCE_FILES)
  add_library(georeloc STATIC ${GEORELOC_SOURCE_FILES})
  set(_GEORELOC_LINK_SCOPE PUBLIC)
  set(_GEORELOC_INCLUDE_SCOPE PUBLIC)
else()
  add_library(georeloc INTERFACE)
  set(_GEORELOC_LINK_SCOPE INTERFACE)
  set(_GEORELOC_INCLUDE_SCOPE INTERFACE)
  message(STATUS "Aucune source trouvee sous src/: la cible georeloc est configuree en bibliotheque d'interface.")
endif()

target_include_directories(georeloc
  ${_GEORELOC_INCLUDE_SCOPE}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bg
)

target_link_libraries(georeloc
  ${_GEORELOC_LINK_SCOPE}
    ${COLMAP_LIB_TARGET}
    Eigen3::Eigen
    SQLite::SQLite3
    ${OpenCV_LIBS}
    Open3D::Open3D
)

set_target_properties(georeloc PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED YES
  CXX_EXTENSIONS NO
)

function(georeloc_apply_warnings target_name)
  target_compile_options(${target_name} PRIVATE
    -Wall -Wextra -Wpedantic
    -Wshadow -Wconversion -Wsign-conversion
  )
endfunction()

if(TARGET georeloc AND NOT GEORELOC_SOURCE_FILES STREQUAL "")
  georeloc_apply_warnings(georeloc)
endif()

foreach(test_src IN LISTS GEORELOC_TEST_SOURCES)
  get_filename_component(test_name "${test_src}" NAME_WE)
  add_executable(${test_name} "${test_src}")
  target_link_libraries(${test_name} PRIVATE georeloc)
  set_target_properties(${test_name} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
  )
  georeloc_apply_warnings(${test_name})
  add_test(NAME ${test_name} COMMAND $<TARGET_FILE:${test_name}>
           WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endforeach()

if(NOT GEORELOC_TEST_SOURCES)
  message(STATUS "Aucun test compilable trouve sous tests/.")
endif()