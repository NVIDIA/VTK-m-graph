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

#include "FilterNode.h"
// vtk-m
#include <vtkm/filter/CellAverage.h>
#include <vtkm/filter/CleanGrid.h>
#include <vtkm/filter/Contour.h>
#include <vtkm/filter/Gradient.h>
#include <vtkm/filter/PointAverage.h>
#include <vtkm/filter/VectorMagnitude.h>

namespace vtkm_anari {
namespace graph {

FilterNode::~FilterNode()
{
  m_datasetInPort.disconnect();
  m_datasetOutPort.disconnectAllDownstreamPorts();
}

InPort *FilterNode::input(const char *name)
{
  if (!std::strcmp(name, m_datasetInPort.name()))
    return &m_datasetInPort;
  return nullptr;
}

OutPort *FilterNode::output(const char *name)
{
  if (!std::strcmp(name, m_datasetOutPort.name()))
    return &m_datasetOutPort;
  return nullptr;
}

NodeType FilterNode::type() const
{
  return NodeType::FILTER;
}

bool FilterNode::isValid() const
{
  return m_datasetInPort.isConnected();
}

// ContourNode //

ContourNode::ContourNode()
{
  addParameter({this, "isovalue", ParameterType::BOUNDED_FLOAT, 0.f});
}

const char *ContourNode::kind() const
{
  return "Contour";
}

void ContourNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE)
    notifyObserver();
}

vtkm::cont::DataSet ContourNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::Range range;
  auto field = ds.GetField(0);
  field.GetRange(&range);

  auto *p = parameter("isovalue");
  p->setMinMax<float>(range.Min, range.Max, range.Center());

  vtkm::filter::Contour filter;
  filter.SetIsoValue(p->valueAs<float>());
  filter.SetActiveField(field.GetName());
  return filter.Execute(ds);
}

// GradientNode //

const char *GradientNode::kind() const
{
  return "Gradient";
}

vtkm::cont::DataSet GradientNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::Gradient filter;
  filter.SetActiveField(ds.GetField(0).GetName());
  filter.SetOutputFieldName("Gradient");
  return filter.Execute(ds);
}

// CleanGridNode //

const char *CleanGridNode::kind() const
{
  return "CleanGrid";
}

vtkm::cont::DataSet CleanGridNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::CleanGrid filter;
  return filter.Execute(ds);
}

// VectorMagnitudeNode //

const char *VectorMagnitudeNode::kind() const
{
  return "VectorMagnitude";
}

vtkm::cont::DataSet VectorMagnitudeNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::VectorMagnitude filter;
  if (ds.GetNumberOfFields() == 0)
    filter.SetUseCoordinateSystemAsField(true);
  else
    filter.SetActiveField(ds.GetField(0).GetName());
  return filter.Execute(ds);
}

// PointAverageNode //

const char *PointAverageNode::kind() const
{
  return "PointAverage";
}

vtkm::cont::DataSet PointAverageNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::PointAverage filter;
  if (ds.GetNumberOfFields() == 0)
    filter.SetUseCoordinateSystemAsField(true);
  else
    filter.SetActiveField(ds.GetField(0).GetName());
  return filter.Execute(ds);
}

// CellAverageNode //

const char *CellAverageNode::kind() const
{
  return "CellAverage";
}

vtkm::cont::DataSet CellAverageNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::CellAverage filter;
  if (ds.GetNumberOfFields() == 0)
    filter.SetUseCoordinateSystemAsField(true);
  else
    filter.SetActiveField(ds.GetField(0).GetName());
  return filter.Execute(ds);
}

} // namespace graph
} // namespace vtkm_anari
