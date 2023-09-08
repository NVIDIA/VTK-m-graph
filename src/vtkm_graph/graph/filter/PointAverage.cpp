// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/field_conversion/PointAverage.h>

namespace vtkm {
namespace graph {

const char *PointAverageNode::kind() const
{
  return "PointAverage";
}

cont::DataSet PointAverageNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::field_conversion::PointAverage filter;
  filter.SetFieldsToPass(filter::FieldSelection::Mode::None);
  if (ds.GetNumberOfFields() == 0)
    filter.SetUseCoordinateSystemAsField(true);
  else
    filter.SetActiveField(ds.GetField(0).GetName());

  return filter.Execute(ds);
}

} // namespace graph
} // namespace vtkm
