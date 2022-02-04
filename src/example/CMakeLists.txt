## Copyright 2022 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

project(example)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME}
PRIVATE
  glm::glm
  vtkm_anari
  vtkm_filter
  vtkm_source
)
