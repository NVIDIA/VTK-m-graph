## Copyright 2022 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.17)

## Language setup ##

set(CMAKE_DISABLE_SOURCE_CHANGES ON)
set(CMAKE_DISABLE_IN_SOURCE_BUILD ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE_INIT "Release")

set(CMAKE_INSTALL_RPATH "$ORIGIN")

## Establish project ##

project(vtkm_anari VERSION 0.0.1 LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

include(GNUInstallDirs)

option(BUILD_SHARED_LIBS "Build shared libraries instead of static" ON)
option(BUILD_EXAMPLES "Build example applications" ON)

## Dependencies ##

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

add_library(glm INTERFACE IMPORTED GLOBAL)
target_link_libraries(glm INTERFACE glm::glm)

find_package(anari REQUIRED)
find_package(VTKm 1.7 REQUIRED)

## Add library targets ##

add_subdirectory(src)

## Configure CMake find_package() config files ##

include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
INSTALL_DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION}
)

write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
DESTINATION
  ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION}
)
