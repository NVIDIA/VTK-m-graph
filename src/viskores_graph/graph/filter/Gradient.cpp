// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// viskores
#include <viskores/filter/vector_analysis/Gradient.h>

namespace viskores {
namespace graph {

const char *GradientNode::kind() const
{
  return "Gradient";
}

cont::DataSet GradientNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::vector_analysis::Gradient filter;
  filter.SetFieldsToPass(filter::FieldSelection::Mode::None);
  filter.SetActiveField(ds.GetField(0).GetName());
  filter.SetOutputFieldName("Gradient");

  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
