// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <viskores/filter/field_conversion/CellAverage.h>

namespace viskores {
namespace graph {

const char *CellAverageNode::kind() const
{
  return "CellAverage";
}

cont::DataSet CellAverageNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::field_conversion::CellAverage filter;
  filter.SetFieldsToPass(filter::FieldSelection::Mode::None);
  if (ds.GetNumberOfFields() == 0)
    filter.SetUseCoordinateSystemAsField(true);
  else
    filter.SetActiveField(ds.GetField(0).GetName());
  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
