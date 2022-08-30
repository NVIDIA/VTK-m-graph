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

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/Probe.h>

namespace vtkm_anari {
namespace graph {

// Helper functions ///////////////////////////////////////////////////////////

static vtkm::cont::DataSet removeHiddenFields(vtkm::cont::DataSet ds)
{
  vtkm::cont::DataSet out;

  for (int i = 0; i < ds.GetNumberOfCoordinateSystems(); i++)
    out.AddCoordinateSystem(ds.GetCoordinateSystem(i));

  for (int i = 0; i < ds.GetNumberOfFields(); i++) {
    auto f = ds.GetField(i);
    if (f.GetName() != "HIDDEN")
      out.AddField(f);
  }

  out.SetCellSet(ds.GetCellSet());

  return out;
}

// ProbeNode definitions //////////////////////////////////////////////////////

ProbeNode::ProbeNode()
{
  addParameter({this, "removeHiddenFields", ParameterType::BOOL, true});
}

const char *ProbeNode::kind() const
{
  return "Probe";
}

void ProbeNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "removeHiddenFields"))
      m_removeHiddenFields = p->valueAs<bool>();
  }

  markChanged();
}

InPort *ProbeNode::inputBegin()
{
  return &m_mainDataInPort;
}

size_t ProbeNode::numInput() const
{
  return 2;
}

InPort *ProbeNode::datasetInput()
{
  return &m_mainDataInPort;
}

InPort *ProbeNode::sampleQuantityInput()
{
  return &m_sampleQuantityInPort;
}

bool ProbeNode::isValid() const
{
  return m_mainDataInPort.isConnected() && m_sampleQuantityInPort.isConnected();
}

vtkm::cont::DataSet ProbeNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());
  auto sq = getDataSetFromPort(sampleQuantityInput());

  vtkm::filter::Probe filter;
  filter.SetGeometry(ds);

  if (m_removeHiddenFields)
    return removeHiddenFields(filter.Execute(sq));
  else
    return filter.Execute(sq);
}

} // namespace graph
} // namespace vtkm_anari
