## Copyright 2022 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

include(FetchContent)
function(fetch_project)
  cmake_parse_arguments(FETCH_SOURCE "" "NAME;URL;DESTINATION" "" ${ARGN})

  FetchContent_Populate(${FETCH_SOURCE_NAME}
    URL ${FETCH_SOURCE_URL}
    SOURCE_DIR ${CMAKE_BINARY_DIR}/${FETCH_SOURCE_DESTINATION}
  )

  set(
    "${FETCH_SOURCE_NAME}_LOCATION"
    ${CMAKE_BINARY_DIR}/${FETCH_SOURCE_DESTINATION}
    PARENT_SCOPE
  )
endfunction()

fetch_project(
  NAME glm_src
  URL https://github.com/g-truc/glm/archive/0.9.9.8.zip
  DESTINATION glm
)

set(glm_DIR "${glm_src_LOCATION}/cmake/glm")
find_package(glm REQUIRED)
