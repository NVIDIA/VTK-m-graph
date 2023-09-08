## Copyright 2023 NVIDIA Corporation
## SPDX-License-Identifier: Apache-2.0

include(GenerateExportHeader)

project_add_library()

generate_export_header(${PROJECT_NAME})

set(SOURCES
  ExecutionGraph.cpp

  graph/ActorNode.cpp
  graph/FilterNode.cpp
  graph/MapperNode.cpp
  graph/FieldSelector.cpp
  graph/Node.cpp
  graph/Parameter.cpp
  graph/Port.cpp
  graph/SourceNode.cpp
  graph/UtilityNode.cpp

  graph/TimeStamp.cpp

  graph/filter/CellAverage.cpp
  graph/filter/CleanGrid.cpp
  graph/filter/Contour.cpp
  graph/filter/Gradient.cpp
  graph/filter/PointAverage.cpp
  graph/filter/Probe.cpp
  graph/filter/Slice.cpp
  graph/filter/Streamline.cpp
  graph/filter/SurfaceNormals.cpp
  graph/filter/Tetrahedralize.cpp
  graph/filter/Tube.cpp
  graph/filter/VectorMagnitude.cpp
  graph/filter/VertexClustering.cpp

  graph/mapper/Glyph.cpp
  graph/mapper/Point.cpp
  graph/mapper/Triangle.cpp
  graph/mapper/Volume.cpp

  graph/source/ABC.cpp
  graph/source/Callback.cpp
  graph/source/RandomPoints.cpp
  graph/source/Tangle.cpp
  graph/source/VTKFileReader.cpp

  graph/utility/DataSetToComponents.cpp
  graph/utility/ComponentsToDataSet.cpp
  graph/utility/VTKFileWriter.cpp
)

project_sources(PRIVATE ${SOURCES})

if (VTKm_ENABLE_CUDA)
  enable_language(CUDA)
  set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CUDA)
endif()

set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_ARCHITECTURES OFF)

project_link_libraries(
PUBLIC
  anari::anari
  vtkm::io
  vtkm::anari
PRIVATE
  glm::glm
  vtkm::filter
  vtkm::source
  vtkm::worklet
  vtkm::filter_flow
)

project_include_directories(
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/detail>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

## Install library + targets

set_target_properties(${PROJECT_NAME}
    PROPERTIES VERSION ${PROJECT_VERSION} SOVERSION ${PROJECT_VERSION_MAJOR})

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_Exports
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    NAMELINK_SKIP
  # on Windows put the dlls into bin
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  # ... and the import lib into the devel package
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(EXPORT ${PROJECT_NAME}_Exports
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}-${PROJECT_VERSION}
  NAMESPACE ${PROJECT_NAME}::
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    NAMELINK_ONLY
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

## Install headers ##

install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
    PATTERN *.h
    PATTERN vtkm EXCLUDE
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}_export.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
)
