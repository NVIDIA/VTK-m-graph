// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/resampling/Probe.h>

namespace vtkm {
namespace graph {

// Helper functions ///////////////////////////////////////////////////////////

static cont::DataSet removeHiddenFields(cont::DataSet ds)
{
  cont::DataSet out;

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

cont::DataSet ProbeNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());
  auto sq = getDataSetFromPort(sampleQuantityInput());

  filter::resampling::Probe filter;
  filter.SetGeometry(ds);

  if (m_removeHiddenFields)
    return removeHiddenFields(filter.Execute(sq));
  else
    return filter.Execute(sq);
}

} // namespace graph
} // namespace vtkm
