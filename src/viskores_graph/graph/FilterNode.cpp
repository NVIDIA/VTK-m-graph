// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "FilterNode.h"
#include "SourceNode.h"

namespace viskores {
namespace graph {

FilterNode::~FilterNode()
{
  m_datasetInPort.disconnect();
  m_datasetOutPort.disconnectAllDownstreamPorts();
}

InPort *FilterNode::inputBegin()
{
  return &m_datasetInPort;
}

size_t FilterNode::numInput() const
{
  return 1;
}

OutPort *FilterNode::outputBegin()
{
  return &m_datasetOutPort;
}

size_t FilterNode::numOutput() const
{
  return 1;
}

InPort *FilterNode::datasetInput()
{
  return &m_datasetInPort;
}

NodeType FilterNode::type() const
{
  return NodeType::FILTER;
}

bool FilterNode::isValid() const
{
  return m_datasetInPort.isConnected();
}

void FilterNode::update()
{
  const bool valid = isValid();
  const bool update = needsUpdate();

  if (!valid && update) {
    m_datasetOutPort.unsetValue();
    markUpdated();
  }

  if (!valid || !update)
    return;

  auto dataset = execute();
  m_datasetOutPort.setValue(dataset);
  setSummaryText(getSummaryString(dataset));
  markUpdated();
}

bool FilterNode::needsUpdate()
{
  return Node::needsUpdate() || m_datasetInPort.connectionHasNewValue();
}

} // namespace graph
} // namespace viskores
