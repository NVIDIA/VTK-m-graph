## Copyright 2023 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

option(BUILD_BENCHMARKS "Build benchmark app" OFF)
if (NOT BUILD_BENCHMARKS)
  return()
endif()

project(benchmark)

find_package(benchmark REQUIRED)

add_executable(${PROJECT_NAME} benchmark.cpp)
target_link_libraries(${PROJECT_NAME}
PRIVATE
  vtkm_source
  vtkm_filter
  vtkm_anari
  vtkm_rendering
  benchmark::benchmark
  stb_image
)
