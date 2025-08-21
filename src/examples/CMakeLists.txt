## Copyright 2023-2024 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

if (Viskores_ENABLE_CUDA)
  enable_language(CUDA)
endif()

function(set_viskores_source_language)
  if (Viskores_ENABLE_CUDA)
    set_source_files_properties(${ARGN} PROPERTIES LANGUAGE CUDA)
  endif()
endfunction()

add_subdirectory(node_editor)
add_subdirectory(simple)
