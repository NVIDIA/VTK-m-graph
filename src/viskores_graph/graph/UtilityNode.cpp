// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "UtilityNode.h"

namespace viskores {
namespace graph {

UtilityNode::UtilityNode(bool primary) : Node(primary)
{
  // no-op
}

NodeType UtilityNode::type() const
{
  return NodeType::UTILITY;
}

bool UtilityNode::isValid() const
{
  return true;
}

} // namespace graph
} // namespace viskores
