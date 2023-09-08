// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../MapperNode.h"
// vtk-m
#include <vtkm/interop/anari/ANARIMapperTriangles.h>

namespace vtkm {
namespace graph {

TriangleMapperNode::TriangleMapperNode()
{
  addParameter({this, "calculate normals", ParameterType::BOOL, false});
}

const char *TriangleMapperNode::kind() const
{
  return "TriangleMapper";
}

void TriangleMapperNode::parameterChanged(
    Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_MINMAX)
    return;

  if (!std::strcmp(p->name(), "calculate normals") && m_mapper) {
    ((interop::anari::ANARIMapperTriangles *)m_mapper)
        ->SetCalculateNormals(p->valueAs<bool>());
  }

  MapperNode::parameterChanged(p, type);
}

void TriangleMapperNode::addMapperToScene(
    interop::anari::ANARIScene &scene, interop::anari::ANARIActor a)
{
  m_scene = &scene;
  m_mapper = &scene.AddMapper(
      interop::anari::ANARIMapperTriangles(scene.GetDevice(), a, name()));
}

} // namespace graph
} // namespace vtkm
