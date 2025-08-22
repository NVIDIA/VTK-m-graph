// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../SourceNode.h"
// viskores
#include <viskores/source/Tangle.h>

namespace viskores {
namespace graph {

TangleSourceNode::TangleSourceNode()
{
  addParameter({this, "size", ParameterType::BOUNDED_INT, 64})
      ->setMinMax(8, 256, 64);
}

const char *TangleSourceNode::kind() const
{
  return "TangleSource";
}

void TangleSourceNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_MINMAX)
    return;

  markChanged();
}

cont::DataSet TangleSourceNode::execute()
{
  auto size = parameter("size")->valueAs<int>();
  auto tangle = source::Tangle();
  tangle.SetPointDimensions(Id3{size});
  return tangle.Execute();
}

} // namespace graph
} // namespace viskores