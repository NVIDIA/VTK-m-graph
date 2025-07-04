## Copyright 2023-2024 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

if (VTKm_ENAGLE_CUDA)
  enable_language(CUDA)
endif()

function(set_vtkm_source_language)
  if (VTKm_ENABLE_CUDA)
    set_source_files_properties(${ARGN} PROPERTIES LANGUAGE CUDA)
  endif()
endfunction()

add_subdirectory(common)
add_subdirectory(node_editor)
add_subdirectory(simple)
