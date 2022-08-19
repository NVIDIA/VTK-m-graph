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

namespace vtkm_anari {
namespace graph {

DataSetToComponentsNode::DataSetToComponentsNode() : UtilityNode(true)
{
  m_outPorts.reserve(10);
  m_outPorts.emplace_back(PortType::COORDINATE_SYSTEM, "coords", this);
  m_outPorts.emplace_back(PortType::CELLSET, "cells", this);
}

DataSetToComponentsNode::~DataSetToComponentsNode()
{
  m_outPorts.clear(); // destroy these before the node is destroyed
}

const char *DataSetToComponentsNode::kind() const
{
  return "DataSetToComponents";
}

InPort *DataSetToComponentsNode::inputBegin()
{
  return &m_datasetInPort;
}

size_t DataSetToComponentsNode::numInput() const
{
  return 1;
}

OutPort *DataSetToComponentsNode::outputBegin()
{
  return m_outPorts.data();
}

size_t DataSetToComponentsNode::numOutput() const
{
  return m_outPorts.size();
}

void DataSetToComponentsNode::update()
{
  if (!needsUpdate())
    return;

  if (!m_datasetInPort.isConnected()) {
    m_outPorts.resize(2);
    return;
  }

  auto ds = getDataSetFromPort(&m_datasetInPort);

  auto numFields = ds.GetNumberOfFields();
  m_outPorts.resize(numFields + 2);

  for (vtkm::IdComponent i = 0; i < numFields; i++) {
    auto f = ds.GetField(i);
    auto &op = m_outPorts[i + 2];
    if (op.id() == INVALID_ID || op.name() != f.GetName())
      op = OutPort(PortType::FIELD, f.GetName(), this);
  }
}

} // namespace graph
} // namespace vtkm_anari
