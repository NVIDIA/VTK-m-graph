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

namespace vtkm {
namespace graph {

ActorNode::ActorNode()
{
  selector().setFieldNames({});
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

const FieldSelector &ActorNode::cselector() const
{
  return m_selector;
}

FieldSelector &ActorNode::selector()
{
  markChanged();
  return m_selector;
}

void ActorNode::update()
{
  if (!needsUpdate())
    return;
  m_actorPort.setValue(makeActor(getDataSetFromPort(&m_datasetPort)));
  markUpdated();
}

interop::anari::ANARIActor ActorNode::makeActor(cont::DataSet ds)
{
  selector().setFieldNames(ds);
#if 1
  const auto &s = cselector();
  size_t primaryFieldIdx = s.currentField();
  primaryFieldIdx = primaryFieldIdx > 3 ? 0 : primaryFieldIdx;

  auto getField = [&](size_t i) -> cont::Field {
    if (s.numFields() <= i)
      return {};
    std::string fieldName = s.fieldName(i);
    return fieldName == "[none]" ? cont::Field{} : ds.GetField(fieldName);
  };

  cont::UnknownCellSet cellSet = ds.GetCellSet();
  cont::CoordinateSystem coordSys;
  if (ds.GetNumberOfCoordinateSystems() > 0)
    coordSys = ds.GetCoordinateSystem();

  auto actor = interop::anari::ANARIActor(
      cellSet, coordSys, getField(0), getField(1), getField(2), getField(3));
  actor.SetPrimaryFieldIndex(primaryFieldIdx);
#else
  auto actor = interop::anari::ANARIActor(ds);
  actor.SetPrimaryFieldIndex(m_currentField);
#endif
  return actor;
}

} // namespace graph
} // namespace vtkm
