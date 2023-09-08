// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../MapperNode.h"
// vtk-m
#include <vtkm/interop/anari/ANARIMapperVolume.h>

namespace vtkm {
namespace graph {

const char *VolumeMapperNode::kind() const
{
  return "VolumeMapper";
}

void VolumeMapperNode::addMapperToScene(
    interop::anari::ANARIScene &scene, interop::anari::ANARIActor a)
{
  m_scene = &scene;
  m_mapper = &scene.AddMapper(
      interop::anari::ANARIMapperVolume(scene.GetDevice(), a, name()));
}

} // namespace graph
} // namespace vtkm
