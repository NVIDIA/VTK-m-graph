// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// viskores
#include <viskores/filter/contour/Slice.h>
// std
#include <cmath>
#include <cstring>

namespace viskores {
namespace graph {

SliceNode::SliceNode()
{
  addParameter({this, "azimuth", ParameterType::BOUNDED_FLOAT, 0.f})
      ->setMinMax<float>(0.f, 360.f, 0.f);
  addParameter({this, "elevation", ParameterType::BOUNDED_FLOAT, 0.f})
      ->setMinMax<float>(0.f, 360.f, 0.f);
  addParameter({this, "position", ParameterType::BOUNDED_FLOAT, 0.5f})
      ->setMinMax<float>(0.f, 1.f, 0.5f);
}

const char *SliceNode::kind() const
{
  return "Slice";
}

void SliceNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "azimuth"))
      m_azelpos[0] = p->valueAs<float>();
    if (!std::strcmp(p->name(), "elevation"))
      m_azelpos[1] = p->valueAs<float>();
    if (!std::strcmp(p->name(), "position"))
      m_azelpos[2] = p->valueAs<float>();
  }
  markChanged();
}

cont::DataSet SliceNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::contour::Slice filter;
  const auto az = Pi_180<Float32>() * m_azelpos[0];
  const auto el = Pi_180<Float32>() * m_azelpos[1];
  const auto nx = std::sin(az) * std::cos(el);
  const auto ny = std::cos(az) * std::cos(el);
  const auto nz = std::sin(el);
  const auto n = Normal(Vec3f_32(nx, ny, nz));
  const auto coords = ds.GetCoordinateSystem();
  const auto bounds = coords.GetBounds();
  const auto diagonal =
      Vec3f(bounds.X.Length(), bounds.Y.Length(), bounds.Z.Length());
  const auto c = static_cast<Vec3f_32>(
      bounds.Center() + ((m_azelpos[2] - 0.5f) * diagonal * n));
  filter.SetImplicitFunction(Plane({c[0], c[1], c[2]}, n));

  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
