// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/geometry_refinement/VertexClustering.h>

namespace vtkm {
namespace graph {

VertexClusteringNode::VertexClusteringNode()
{
  addParameter({this, "divisions", ParameterType::BOUNDED_INT, 32})
      ->setMinMax<int>(8, 512, 32);
}

const char *VertexClusteringNode::kind() const
{
  return "VertexClustering";
}

void VertexClusteringNode::parameterChanged(
    Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE)
    markChanged();
}

cont::DataSet VertexClusteringNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::geometry_refinement::VertexClustering filter;
  auto div = parameter("divisions")->valueAs<int>();
  filter.SetNumberOfDivisions(Id3(div, div, div));

  return filter.Execute(ds);
}

} // namespace graph
} // namespace vtkm
