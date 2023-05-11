## Copyright 2023 NVIDIA Corporation
## SPDX-License-Identifier: Unlicense

option(BUILD_INTERACTIVE_EXAMPLE "Build interactive example app" OFF)
if (NOT BUILD_INTERACTIVE_EXAMPLE)
  return()
endif()

project(node_viewer)

project_add_executable()
project_sources(
PRIVATE
  main.cpp
  NodeEditor.cpp
  NodeInfoWindow.cpp
  ui_layout.cpp
)
set_vtkm_source_language(main.cpp)
project_link_libraries(PRIVATE anari::anari_viewer vtkm_graph)

set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES OFF)
