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

#include "ActorNode.h"
#include "FilterNode.h"
#include "SourceNode.h"

namespace vtkm_anari {
namespace graph {

ActorNode::ActorNode()
{
  setFieldNames({});
}

ActorNode::~ActorNode()
{
  m_datasetPort.disconnect();
  m_actorPort.disconnectAllDownstreamPorts();
}

const char *ActorNode::kind() const
{
  return "Actor";
}

InPort *ActorNode::inputBegin()
{
  return &m_datasetPort;
}

size_t ActorNode::numInput() const
{
  return 1;
}

OutPort *ActorNode::outputBegin()
{
  return &m_actorPort;
}

size_t ActorNode::numOutput() const
{
  return 1;
}

NodeType ActorNode::type() const
{
  return NodeType::ACTOR;
}

bool ActorNode::isValid() const
{
  return m_datasetPort.isConnected();
}

void ActorNode::setFieldNames(vtkm::cont::DataSet ds)
{
  m_fields.clear();
  for (size_t i = 0; i < ds.GetNumberOfFields(); i++)
    m_fields.push_back(ds.GetField(i).GetName());
  m_fields.push_back("[none]");
  if (m_currentField >= m_fields.size())
    m_currentField = 0;
  markChanged();
}

size_t ActorNode::numFields() const
{
  return m_fields.size();
}

const char *ActorNode::fieldName(size_t i) const
{
  return m_fields[i].c_str();
}

size_t ActorNode::getCurrentField() const
{
  return m_currentField;
}

void ActorNode::setCurrentField(size_t i)
{
  m_currentField = i;
  markChanged();
}

void ActorNode::update()
{
  if (!needsUpdate())
    return;

  m_actor = {};

  auto *p = &m_datasetPort;
  if (!p->isConnected())
    return;

  vtkm::cont::DataSet ds;

  auto *node = p->other()->node();
  if (node->type() == NodeType::FILTER)
    ds = ((FilterNode *)node)->dataset();
  else if (node->type() == NodeType::SOURCE)
    ds = ((SourceNode *)node)->dataset();

  m_actor = makeActor(ds);

  markUpdated();
}

ANARIActor ActorNode::actor()
{
  update();
  return m_actor;
}

ANARIActor ActorNode::makeActor(vtkm::cont::DataSet ds)
{
  setFieldNames(ds);
  if (ds.GetNumberOfFields() == 0 || m_currentField == (m_fields.size() - 1))
    return ANARIActor(ds.GetCellSet(), ds.GetCoordinateSystem(), {});
  else {
    return ANARIActor(
        ds.GetCellSet(), ds.GetCoordinateSystem(), ds.GetField(m_currentField));
  }
}

} // namespace graph
} // namespace vtkm_anari
