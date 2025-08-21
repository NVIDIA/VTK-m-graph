// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../MapperNode.h"
// vtk-m
#include <viskores/interop/anari/ANARIMapperPoints.h>

namespace viskores {
namespace graph {

const char *PointMapperNode::kind() const
{
  return "PointMapper";
}

void PointMapperNode::addMapperToScene(
    interop::anari::ANARIScene &scene, interop::anari::ANARIActor a)
{
  m_scene = &scene;
  m_mapper = &scene.AddMapper(
      interop::anari::ANARIMapperPoints(scene.GetDevice(), a, uniqueName()));
}

} // namespace graph
} // namespace viskores
