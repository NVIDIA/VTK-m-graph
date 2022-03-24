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

#include "ANARIMapperPoints.h"
#include "vtkm/SphereExtractor.h"
// anari + glm
#include <anari/anari_cpp/ext/glm.h>
// vtk-m
#include <vtkm/worklet/WorkletMapField.h>

namespace vtkm_anari {

// Worklets ///////////////////////////////////////////////////////////////////

class ExtractPointPositions : public vtkm::worklet::WorkletMapField
{
 public:
  VTKM_CONT
  ExtractPointPositions() = default;

  using ControlSignature = void(FieldIn, WholeArrayIn, WholeArrayOut);
  using ExecutionSignature = void(InputIndex, _1, _2, _3);

  template <typename InPointPortalType, typename OutPointPortalType>
  VTKM_EXEC void operator()(const vtkm::Id out_idx,
      const vtkm::Id in_idx,
      const InPointPortalType &points,
      OutPointPortalType &out) const
  {
    out.Set(out_idx, static_cast<vtkm::Vec3f_32>(points.Get(in_idx)));
  }
};

// Helper functions ///////////////////////////////////////////////////////////

static vtkm::cont::ArrayHandle<vtkm::Vec3f_32> unpackPoints(
    vtkm::cont::ArrayHandle<vtkm::Id> points,
    vtkm::cont::CoordinateSystem coords)
{
  const auto numPoints = points.GetNumberOfValues();

  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> vertices;
  vertices.Allocate(numPoints);

  vtkm::worklet::DispatcherMapField<ExtractPointPositions>().Invoke(
      points, coords, vertices);

  return vertices;
}

// ANARIMapperPoints definitions //////////////////////////////////////////////

ANARIMapperPoints::ANARIMapperPoints(
    anari::Device device, const ANARIActor &actor)
    : ANARIMapper(device, actor)
{}

ANARIMapperPoints::~ANARIMapperPoints()
{
  auto d = GetDevice();
  anari::release(d, m_parameters.vertex.position);
  anari::release(d, m_parameters.vertex.radius);
  anari::release(d, m_parameters.vertex.attribute);
}

const PointsParameters &ANARIMapperPoints::Parameters()
{
  constructParameters();
  return m_parameters;
}

anari::Geometry ANARIMapperPoints::MakeANARIGeometry()
{
  constructParameters();
  if (!m_parameters.vertex.position)
    return nullptr;
  auto d = GetDevice();
  auto geometry = anari::newObject<anari::Geometry>(d, "sphere");
  anari::setParameter(
      d, geometry, "vertex.position", m_parameters.vertex.position);
  anari::setParameter(d, geometry, "vertex.radius", m_parameters.vertex.radius);
  anari::setParameter(
      d, geometry, "vertex.attribute0", m_parameters.vertex.attribute);
  anari::commit(d, geometry);
  return geometry;
}

void ANARIMapperPoints::constructParameters()
{
  if (m_parameters.vertex.position)
    return;

  const auto &actor = GetActor();
  const auto &coords = actor.GetCoordinateSystem();
  const auto &cells = actor.GetCellSet();

  vtkm::Bounds coordBounds = coords.GetBounds();
  // set a default radius
  vtkm::Float64 lx = coordBounds.X.Length();
  vtkm::Float64 ly = coordBounds.Y.Length();
  vtkm::Float64 lz = coordBounds.Z.Length();
  vtkm::Float64 mag = vtkm::Sqrt(lx * lx + ly * ly + lz * lz);
  // same as used in vtk ospray
  constexpr vtkm::Float64 heuristic = 500.;
  auto baseRadius = static_cast<vtkm::Float32>(mag / heuristic);

  vtkm::rendering::raytracing::SphereExtractor sphereExtractor;

  sphereExtractor.ExtractCoordinates(coords, baseRadius);

  auto numPoints = sphereExtractor.GetNumberOfSpheres();
  m_parameters.numPrimitives = static_cast<uint32_t>(numPoints);

  if (numPoints == 0)
    printf("NO POINTS GENERATED\n");
  else {
    m_vertices = unpackPoints(sphereExtractor.GetPointIds(), coords);
    m_radii = sphereExtractor.GetRadii();
    vtkm::cont::Token t;
    auto *p = (glm::vec3 *)m_vertices.GetBuffers()->ReadPointerHost(t);
    auto *r = (float *)m_radii.GetBuffers()->ReadPointerHost(t);
    auto d = GetDevice();
    m_parameters.vertex.position = anari::newArray1D(d, p, numPoints);
    m_parameters.vertex.radius = anari::newArray1D(d, r, numPoints);
  }
}

} // namespace vtkm_anari