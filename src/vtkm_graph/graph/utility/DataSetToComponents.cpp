// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../UtilityNode.h"

namespace vtkm {
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
    m_outPorts[0].unsetValue();
    m_outPorts[1].unsetValue();
    markUpdated();
    return;
  }

  auto ds = getDataSetFromPort(&m_datasetInPort);

  auto numFields = ds.GetNumberOfFields();
  m_outPorts.resize(numFields + 2);

  if (ds.GetNumberOfCoordinateSystems() > 0)
    m_outPorts[0].setValue(ds.GetCoordinateSystem());
  m_outPorts[1].setValue(ds.GetCellSet());

  for (IdComponent i = 0; i < numFields; i++) {
    auto f = ds.GetField(i);
    auto &op = m_outPorts[i + 2];
    if (op.id() == INVALID_ID || op.name() != f.GetName())
      op = OutPort(PortType::FIELD, f.GetName(), this);
    op.setValue(f);
  }

  markUpdated();
}

} // namespace graph
} // namespace vtkm
