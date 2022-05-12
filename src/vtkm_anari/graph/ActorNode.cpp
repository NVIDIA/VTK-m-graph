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

namespace vtkm_anari::graph {

ActorNode::~ActorNode()
{
  m_datasetPort.disconnect();
  m_actorPort.disconnectAllDownstreamPorts();
}

const char *ActorNode::kind() const
{
  return "Actor";
}

InPort *ActorNode::input(const char *name)
{
  if (!std::strcmp(name, m_datasetPort.name()))
    return &m_datasetPort;
  return nullptr;
}

OutPort *ActorNode::output(const char *name)
{
  if (!std::strcmp(name, m_actorPort.name()))
    return &m_actorPort;
  return nullptr;
}

NodeType ActorNode::type() const
{
  return NodeType::ACTOR;
}

bool ActorNode::isValid() const
{
  return m_datasetPort.isConnected();
}

ANARIActor ActorNode::makeActor(vtkm::cont::DataSet ds)
{
  if (ds.GetNumberOfFields() == 0)
    return ANARIActor(ds.GetCellSet(), ds.GetCoordinateSystem(), {});
  else
    return ANARIActor(
        ds.GetCellSet(), ds.GetCoordinateSystem(), ds.GetField(0));
}

} // namespace vtkm_anari::graph
