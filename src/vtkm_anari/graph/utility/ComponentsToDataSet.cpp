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

#include "../UtilityNode.h"
// std
#include <algorithm>

namespace vtkm_anari {
namespace graph {

ComponentsToDataSetNode::ComponentsToDataSetNode() : UtilityNode(true)
{
  m_inPorts.reserve(10);
  m_inPorts.emplace_back(PortType::COORDINATE_SYSTEM, "coords", this);
  m_inPorts.emplace_back(PortType::CELLSET, "cells", this);
  m_inPorts.emplace_back(PortType::FIELD, "field", this);
}

ComponentsToDataSetNode::~ComponentsToDataSetNode()
{
  m_inPorts.clear();
}

const char *ComponentsToDataSetNode::kind() const
{
  return "ComponentsToDataSet";
}

InPort *ComponentsToDataSetNode::inputBegin()
{
  return m_inPorts.data();
}

size_t ComponentsToDataSetNode::numInput() const
{
  return m_inPorts.size();
}

OutPort *ComponentsToDataSetNode::outputBegin()
{
  return &m_datasetOutPort;
}

size_t ComponentsToDataSetNode::numOutput() const
{
  return 1;
}

void ComponentsToDataSetNode::update()
{
  if (!needsUpdate())
    return;

  m_dataset = {};

  auto pt = std::stable_partition(m_inPorts.begin() + 2,
      m_inPorts.end(),
      [](auto &p) { return p.isConnected(); });

  if (pt == m_inPorts.end())
    m_inPorts.emplace_back(PortType::FIELD, "field", this);
  else
    m_inPorts.erase(pt + 1, m_inPorts.end());

  const bool valid = m_inPorts[0].isConnected() && m_inPorts[1].isConnected();
  if (!valid)
    return;

#if 0
  auto ds = getDataSetFromPort(&m_datasetInPort);

  for (vtkm::IdComponent i = 0; i < ds.GetNumberOfFields(); i++) {
    auto f = ds.GetField(i);
    m_outPorts.emplace_back(PortType::FIELD, f.GetName(), this);
  }
#endif
}

} // namespace graph
} // namespace vtkm_anari
