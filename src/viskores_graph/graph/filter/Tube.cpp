// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// viskores
#include <viskores/filter/geometry_refinement/Tube.h>
// std
#include <cmath>
#include <cstring>

namespace viskores {
namespace graph {

TubeNode::TubeNode()
{
  addParameter({this, "sides", ParameterType::BOUNDED_INT, 7})
      ->setMinMax<int>(3, 32, 7);
  addParameter({this, "radius", ParameterType::FLOAT, 0.f});
  addParameter({this, "cap", ParameterType::BOOL, false});
}

const char *TubeNode::kind() const
{
  return "Tube";
}

void TubeNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "sides"))
      m_sides = p->valueAs<int>();
    if (!std::strcmp(p->name(), "radius"))
      m_radius = p->valueAs<float>();
    if (!std::strcmp(p->name(), "cap"))
      m_cap = p->valueAs<bool>();
  }

  markChanged();
}

cont::DataSet TubeNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  float radius = m_radius;

  if (radius == 0.f) {
    const Bounds bounds = ds.GetCoordinateSystem().GetBounds();
    const float diagonal =
        std::sqrt(std::pow((float)(bounds.X.Max - bounds.X.Min), 2.f)
            + std::pow((float)(bounds.Y.Max - bounds.Y.Min), 2.f)
            + std::pow((float)(bounds.Z.Max - bounds.Z.Min), 2.f));
    radius = diagonal / 1000.f;
  }

  filter::geometry_refinement::Tube filter;
  filter.SetRadius(radius);
  filter.SetNumberOfSides(m_sides);
  filter.SetCapping(m_cap);

  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
