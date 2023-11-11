## Copyright 2023 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

project(tf_editor)
project_add_library(INTERFACE)
project_sources(INTERFACE ${CMAKE_CURRENT_LIST_DIR}/TransferFunctionEditor.cpp)
project_include_directories(INTERFACE ${CMAKE_CURRENT_LIST_DIR})
