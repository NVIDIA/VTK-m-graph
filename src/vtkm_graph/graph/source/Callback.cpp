// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../SourceNode.h"

namespace vtkm {
namespace graph {

CallbackSourceNode::CallbackSourceNode(SourceNodeCallback cb)
{
  setCallback(cb);
}

const char *CallbackSourceNode::kind() const
{
  return "CallbackSource";
}

void CallbackSourceNode::setCallback(SourceNodeCallback cb)
{
  m_callback = cb;
}

void CallbackSourceNode::markChanged()
{
  Node::markChanged();
}

cont::DataSet CallbackSourceNode::execute()
{
  return m_callback();
}

} // namespace graph
} // namespace vtkm