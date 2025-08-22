// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../MapperNode.h"
// viskores
#include <viskores/interop/anari/ANARIMapperGlyphs.h>

namespace viskores {
namespace graph {

const char *GlyphMapperNode::kind() const
{
  return "GlyphMapper";
}

void GlyphMapperNode::addMapperToScene(
    interop::anari::ANARIScene &scene, interop::anari::ANARIActor a)
{
  m_scene = &scene;
  m_mapper = &scene.AddMapper(
      interop::anari::ANARIMapperGlyphs(scene.GetDevice(), a, uniqueName()));
}

} // namespace graph
} // namespace viskores
