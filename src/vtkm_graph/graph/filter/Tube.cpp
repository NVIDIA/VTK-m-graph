/*
 * Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/geometry_refinement/Tube.h>
// std
#include <cmath>
#include <cstring>

namespace vtkm {
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
} // namespace vtkm
