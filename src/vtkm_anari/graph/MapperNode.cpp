/*
 * Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "MapperNode.h"
#include "../ANARIMapperGlyphs.h"
#include "../ANARIMapperPoints.h"
#include "../ANARIMapperTriangles.h"
#include "../ANARIMapperVolume.h"

namespace vtkm_anari::graph {

MapperNode::~MapperNode()
{
  m_actorPort.disconnect();
}

InPort *MapperNode::input(const char *name)
{
  if (!std::strcmp(name, m_actorPort.name()))
    return &m_actorPort;
  return nullptr;
}

NodeType MapperNode::type() const
{
  return NodeType::MAPPER;
}

bool MapperNode::isValid() const
{
  return m_actorPort.isConnected();
}

bool MapperNode::isVisible() const
{
  return m_visible;
}

void MapperNode::setVisible(bool show)
{
  m_visible = show;
}

// VolumeMapperNode //

const char *VolumeMapperNode::kind() const
{
  return "VolumeMapper";
}

void VolumeMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperVolume(scene.GetDevice(), a));
}

// TriangleMapperNode //

const char *TriangleMapperNode::kind() const
{
  return "TriangleMapper";
}

void TriangleMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperTriangles(scene.GetDevice(), a));
}

// PointMapperNode //

const char *PointMapperNode::kind() const
{
  return "PointMapper";
}

void PointMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperPoints(scene.GetDevice(), a));
}

// GlyphMapperNode //

const char *GlyphMapperNode::kind() const
{
  return "GlyphMapper";
}

void GlyphMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperGlyphs(scene.GetDevice(), a));
}

} // namespace vtkm_anari::graph