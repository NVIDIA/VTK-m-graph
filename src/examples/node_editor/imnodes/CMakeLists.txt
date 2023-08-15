## Copyright 2023 NVIDIA Corporation
## SPDX-License-Identifier: Unlicense

project(anari_viewer_imnodes VERSION ${PROJECT_VERSION} LANGUAGES CXX)

anari_sdk_fetch_project(
  NAME ${PROJECT_NAME}
  URL https://github.com/Nelarius/imnodes/archive/refs/tags/v0.5.zip
  MD5 bebc93614fb55fc5924191b3c5320026
)

project_add_library(INTERFACE)

project_sources(
INTERFACE
  $<BUILD_INTERFACE:${anari_viewer_imnodes_LOCATION}/imnodes.cpp>
)

project_include_directories(
INTERFACE
  $<BUILD_INTERFACE:${anari_viewer_imnodes_LOCATION}>
)
