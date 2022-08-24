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
#include "ActorNode.h"

namespace vtkm_anari {
namespace graph {

MapperNode::MapperNode() : Node(true)
{
  // no-op
}

MapperNode::~MapperNode()
{
  m_actorPort.disconnect();
  m_scene->RemoveMapper(m_scene->GetMapperIndexByName(name()));
}

InPort *MapperNode::inputBegin()
{
  return &m_actorPort;
}

size_t MapperNode::numInput() const
{
  return 1;
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
  if (m_scene)
    m_scene->SetMapperVisible(m_scene->GetMapperIndexByName(name()), show);
  notifyObserver();
}

bool MapperNode::isMapperEmpty() const
{
  return m_mapper->GroupIsEmpty();
}

ANARIMapper *MapperNode::getMapper() const
{
  return m_mapper;
}

void MapperNode::update()
{
  if (!m_mapper || !isVisible() || !needsUpdate())
    return;

  if (!m_actorPort.isConnected()) {
    m_mapper->SetActor({});
    markUpdated();
    return;
  }

  auto *p = m_actorPort.other();
  p->node()->update();
  m_mapper->SetActor(p->getValue().Get<ANARIActor>());
  markUpdated();
}

} // namespace graph
} // namespace vtkm_anari
