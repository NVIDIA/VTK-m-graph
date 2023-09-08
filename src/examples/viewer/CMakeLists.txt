## Copyright 2023 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

option(BUILD_EXAMPLE_VIEWER "Build interactive example app" OFF)
if (NOT BUILD_EXAMPLE_VIEWER)
  return()
endif()

project(viewer)

project_add_executable()
project_sources(PRIVATE GraphControlsWindow.cpp main.cpp ui_layout.cpp)
set_vtkm_source_language(main.cpp)
project_link_libraries(PRIVATE anari::anari_viewer vtkm_graph)

set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES OFF)
