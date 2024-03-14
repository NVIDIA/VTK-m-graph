// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../SourceNode.h"
// vtk-m
#include <vtkm/cont/DataSetBuilderExplicit.h>
// std
#include <random>

namespace vtkm {
namespace graph {

const char *RandomPointsSourceNode::kind() const
{
  return "RandomPointsSource";
}

cont::DataSet RandomPointsSourceNode::execute()
{
  constexpr int numSpheres = 1e4;

  std::mt19937 rng;
  rng.seed(0);
  std::normal_distribution<float> vert_dist(0.f, 4.f);

  cont::DataSetBuilderExplicitIterative builder;

  for (int i = 0; i < numSpheres; i++) {
    builder.AddPoint(vert_dist(rng), vert_dist(rng), vert_dist(rng));
    builder.AddCell(CELL_SHAPE_VERTEX);
    builder.AddCellPoint(i);
  }

  return builder.Create();
}

} // namespace graph
} // namespace vtkm
