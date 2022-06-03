/*
 * Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <vtkm/filter/Slice.h>
// std
#include <cmath>
#include <cstring>

namespace vtkm_anari {
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

vtkm::cont::DataSet SliceNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  vtkm::filter::Slice filter;
  const auto az = vtkm::Pi_180<vtkm::Float32>() * m_azelpos[0];
  const auto el = vtkm::Pi_180<vtkm::Float32>() * m_azelpos[1];
  const auto nx = std::sin(az) * std::cos(el);
  const auto ny = std::cos(az) * std::cos(el);
  const auto nz = std::sin(el);
  const auto n = vtkm::Normal(vtkm::Vec3f_32(nx, ny, nz));
  const auto coords = ds.GetCoordinateSystem();
  const auto bounds = coords.GetBounds();
  const auto diagonal =
      vtkm::Vec3f(bounds.X.Length(), bounds.Y.Length(), bounds.Z.Length());
  const auto c = static_cast<vtkm::Vec3f_32>(
      bounds.Center() + ((m_azelpos[2] - 0.5f) * diagonal * n));
  filter.SetImplicitFunction(vtkm::Plane({c[0], c[1], c[2]}, n));

  return filter.Execute(ds);
}

} // namespace graph
} // namespace vtkm_anari
