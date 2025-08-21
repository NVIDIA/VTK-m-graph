// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../UtilityNode.h"
// std
#include <algorithm>

namespace viskores {
namespace graph {

ComponentsToDataSetNode::ComponentsToDataSetNode() : UtilityNode(true)
{
  m_inPorts.reserve(6);
  m_inPorts.emplace_back(PortType::COORDINATE_SYSTEM, "coords", this);
  m_inPorts.emplace_back(PortType::CELLSET, "cells", this);
  m_inPorts.emplace_back(PortType::FIELD, "field1", this);
  m_inPorts.emplace_back(PortType::FIELD, "field2", this);
  m_inPorts.emplace_back(PortType::FIELD, "field3", this);
  m_inPorts.emplace_back(PortType::FIELD, "field4", this);
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

  const bool valid = m_inPorts[0].isConnected() && m_inPorts[1].isConnected();
  if (!valid) {
    m_datasetOutPort.unsetValue();
    markUpdated();
    return;
  }

  cont::DataSet ds;

  ds.AddCoordinateSystem(getCoordinateSystemFromPort(&m_inPorts[0]));
  ds.SetCellSet(getCellSetFromPort(&m_inPorts[1]));
  std::for_each(m_inPorts.begin() + 2, m_inPorts.end(), [&](auto &p) {
    if (p.isConnected())
      ds.AddField(getFieldFromPort(&p));
  });

  m_datasetOutPort.setValue(ds);
  setSummaryText(getSummaryString(ds));
  markUpdated();
}

} // namespace graph
} // namespace viskores
