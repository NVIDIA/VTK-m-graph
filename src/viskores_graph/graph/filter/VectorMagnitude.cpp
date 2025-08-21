// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <viskores/filter/vector_analysis/VectorMagnitude.h>

namespace viskores {
namespace graph {

const char *VectorMagnitudeNode::kind() const
{
  return "VectorMagnitude";
}

cont::DataSet VectorMagnitudeNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::vector_analysis::VectorMagnitude filter;
  filter.SetFieldsToPass(filter::FieldSelection::Mode::None);
  if (ds.GetNumberOfFields() == 0)
    filter.SetUseCoordinateSystemAsField(true);
  else
    filter.SetActiveField(ds.GetField(0).GetName());

  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
