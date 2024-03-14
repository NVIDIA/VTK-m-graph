// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/geometry_refinement/Tetrahedralize.h>

namespace vtkm {
namespace graph {

const char *TetrahedralizeNode::kind() const
{
  return "Tetrahedralize";
}

cont::DataSet TetrahedralizeNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());
  return filter::geometry_refinement::Tetrahedralize().Execute(ds);
}

} // namespace graph
} // namespace vtkm
