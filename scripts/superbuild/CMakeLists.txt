## Copyright 2023-2024 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.17)

cmake_policy(SET CMP0135 NEW)

if(NOT CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX
      "${CMAKE_BINARY_DIR}/install"
      CACHE STRING "Final install location." FORCE)
endif()

set(CMAKE_BUILD_TYPE_INIT "Release")

project(build_vtkm_graph)
include(ExternalProject)
include(ProcessorCount)
include(CMakePrintHelpers)
include(macros.cmake)

ProcessorCount(PROCESSOR_COUNT)
set(NUM_BUILD_JOBS ${PROCESSOR_COUNT} CACHE STRING "Number of build jobs '-j <n>'")
set(DEFAULT_BUILD_COMMAND cmake --build . --config release -j ${NUM_BUILD_JOBS})

get_filename_component(INSTALL_DIR_ABSOLUTE
  ${CMAKE_INSTALL_PREFIX} ABSOLUTE BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})

## Build VTK-m-graph dependencies ##

#option(GET_TBB "Download TBB during build process, otherwise TBB_ROOT must be set" OFF)
#if (GET_TBB)
#  include(get_tbb.cmake)
#  set(TBB_TARGET tbb)
#else()
#  if (NOT DEFINED TBB_ROOT)
#    message(FATAL_ERROR "TBB_ROOT must be specified if GET_TBB=OFF")
#  endif()
#endif()

build_subproject(
  NAME ANARI-SDK
  URL "https://github.com/KhronosGroup/ANARI-SDK/archive/refs/heads/next_release.zip"
  BUILD_ARGS
    -DBUILD_CTS=OFF
    -DBUILD_EXAMPLES=OFF
    -DBUILD_HELIDE_DEVICE=OFF
    -DBUILD_REMOTE_DEVICE=OFF
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_TESTING=OFF
    -DBUILD_VIEWER=OFF
    -DINSTALL_CODE_GEN_SCRIPTS=OFF
    -DINSTALL_VIEWER_LIBRARY=OFF
)

build_subproject(
  NAME VTK-m
  URL "https://github.com/Kitware/VTK-m/archive/refs/heads/master.zip"
  BUILD_ARGS
    -DVTKm_ENABLE_ANARI=ON
    -DVTKm_ENABLE_BENCHMARKS=OFF
    -DVTKm_ENABLE_CPACK=ON
    -DVTKm_ENABLE_CUDA=OFF
    -DVTKm_ENABLE_DOCUMENTATION=OFF
    -DVTKm_ENABLE_EXAMPLES=OFF
    -DVTKm_ENABLE_HDF5_IO=OFF
    -DVTKm_ENABLE_KOKKOS=OFF
    -DVTKm_ENABLE_MPI=OFF
    -DVTKm_ENABLE_OPENMP=ON
    -DVTKm_ENABLE_TESTING=OFF
    -DVTKm_ENABLE_TUTORIALS=OFF
  DEPENDS_ON ANARI-SDK
)

## Build VTK-m-graph itself ##

set(SUBPROJECT_NAME vtkm_graph)
setup_subproject_path_vars(${SUBPROJECT_NAME})
ExternalProject_Add(${SUBPROJECT_NAME}
  PREFIX ${SUBPROJECT_NAME}
  DOWNLOAD_DIR ${SUBPROJECT_NAME}
  STAMP_DIR ${SUBPROJECT_STAMP_PATH}
  SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/../..
  BINARY_DIR ${SUBPROJECT_BUILD_PATH}
  LIST_SEPARATOR |
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
    -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
    -DCMAKE_INSTALL_PREFIX:PATH=${SUBPROJECT_INSTALL_PATH}
    -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}
  BUILD_COMMAND ${DEFAULT_BUILD_COMMAND}
  BUILD_ALWAYS OFF
)
ExternalProject_Add_StepDependencies(${SUBPROJECT_NAME} configure VTK-m)
