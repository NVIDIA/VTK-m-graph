// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "SourceNode.h"

namespace viskores {
namespace graph {

SourceNode::~SourceNode()
{
  m_datasetPort.disconnectAllDownstreamPorts();
}

OutPort *SourceNode::outputBegin()
{
  return &m_datasetPort;
}

size_t SourceNode::numOutput() const
{
  return 1;
}

NodeType SourceNode::type() const
{
  return NodeType::SOURCE;
}

bool SourceNode::isValid() const
{
  return true;
}

void SourceNode::update()
{
  const bool valid = isValid();
  const bool update = needsUpdate();

  if (!valid && update) {
    m_datasetPort.unsetValue();
    markUpdated();
  }

  if (!valid || !update)
    return;

  auto dataset = execute();
  m_datasetPort.setValue(dataset);
  setSummaryText(getSummaryString(dataset));
  markUpdated();
}

} // namespace graph
} // namespace viskores