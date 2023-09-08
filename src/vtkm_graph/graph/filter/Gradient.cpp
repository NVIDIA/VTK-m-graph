// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/vector_analysis/Gradient.h>

namespace vtkm {
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
} // namespace vtkm
