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

#include "ConnectorNode.h"
#include "ActorNode.h"

namespace vtkm {
namespace graph {

ConnectorNode::ConnectorNode() : Node(true)
{
  // no-op
}

ConnectorNode::~ConnectorNode()
{
  m_actorPort.disconnect();
  m_scene->RemoveConnector(m_scene->GetConnectorIndexByName(name()));
}

InPort *ConnectorNode::inputBegin()
{
  return &m_actorPort;
}

size_t ConnectorNode::numInput() const
{
  return 1;
}

NodeType ConnectorNode::type() const
{
  return NodeType::MAPPER;
}

bool ConnectorNode::isValid() const
{
  return m_actorPort.isConnected();
}

bool ConnectorNode::isVisible() const
{
  return m_visible;
}

void ConnectorNode::setVisible(bool show)
{
  m_visible = show;
  if (m_scene)
    m_scene->SetConnectorVisible(m_scene->GetConnectorIndexByName(name()), show);
  notifyObserver();
}

bool ConnectorNode::isConnectorEmpty() const
{
  return m_connector->GroupIsEmpty();
}

interop::anari::ANARIConnector *ConnectorNode::getConnector() const
{
  return m_connector;
}

void ConnectorNode::update()
{
  if (!m_connector || !isVisible() || !needsUpdate())
    return;

  if (!m_actorPort.isConnected()) {
    m_connector->SetActor({});
    markUpdated();
    return;
  }

  auto *p = m_actorPort.other();
  p->node()->update();
  m_connector->SetActor(p->value().Get<interop::anari::ANARIActor>());
  markUpdated();
}

} // namespace graph
} // namespace vtkm
