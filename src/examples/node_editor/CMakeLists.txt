## Copyright 2023 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

option(BUILD_EXAMPLE_EDITOR "Build node editor example app" OFF)
if (NOT BUILD_EXAMPLE_EDITOR)
  return()
endif()

find_package(anari ${ANARI_VERSION_REQUIRED} REQUIRED COMPONENTS viewer)

add_subdirectory(imnodes)

project(node_editor)

project_add_executable()

project_sources(
PRIVATE
  main.cpp
  NodeEditor.cpp
  NodeInfoWindow.cpp
  ui_layout.cpp
)

set_vtkm_source_language(main.cpp)

project_link_libraries(
PRIVATE
  anari::anari_viewer
  anari_viewer_imnodes
  vtkm_graph
)

project_compile_definitions(PRIVATE -DIMGUI_DEFINE_MATH_OPERATORS)

set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES OFF)
