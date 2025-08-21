// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../MapperNode.h"
// vtk-m
#include <viskores/interop/anari/ANARIMapperTriangles.h>

namespace viskores {
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
      interop::anari::ANARIMapperTriangles(scene.GetDevice(), a, uniqueName()));
}

} // namespace graph
} // namespace viskores
