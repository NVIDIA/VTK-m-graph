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
// anari + glm
#include <anari/anari_cpp/ext/glm.h>
// vtk-m
#include <vtkm/rendering/raytracing/SphereExtractor.h>
#include <vtkm/worklet/WorkletMapField.h>

namespace vtkm_anari {

// Worklets ///////////////////////////////////////////////////////////////////

class ExtractPointPositions : public vtkm::worklet::WorkletMapField
{
 public:
  VTKM_CONT
  ExtractPointPositions() = default;

  typedef void ControlSignature(FieldIn, WholeArrayIn, WholeArrayOut);
  typedef void ExecutionSignature(InputIndex, _1, _2, _3);

  template <typename PointPortalType, typename OutPortalType>
  VTKM_EXEC void operator()(const vtkm::Id out_idx,
      const vtkm::Id in_idx,
      const PointPortalType &points,
      OutPortalType &out) const
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

ANARIMapperPoints::ANARIMapperPoints(anari::Device device, Actor actor)
    : ANARIMapper(device, actor)
{}

ANARIMapperPoints::~ANARIMapperPoints()
{
  anari::release(m_device, m_parameters.vertex.position);
  anari::release(m_device, m_parameters.vertex.radius);
  anari::release(m_device, m_parameters.vertex.attribute);
}

const PointsParameters &ANARIMapperPoints::parameters()
{
  constructParameters();
  return m_parameters;
}

anari::Geometry ANARIMapperPoints::makeGeometry()
{
  constructParameters();
  if (!m_parameters.vertex.position)
    return nullptr;
  auto geometry = anari::newObject<anari::Geometry>(m_device, "sphere");
  anari::setParameter(
      m_device, geometry, "vertex.position", m_parameters.vertex.position);
  anari::setParameter(
      m_device, geometry, "vertex.radius", m_parameters.vertex.radius);
  anari::setParameter(
      m_device, geometry, "vertex.attribute0", m_parameters.vertex.attribute);
  anari::commit(m_device, geometry);
  return geometry;
}

void ANARIMapperPoints::constructParameters()
{
  if (m_parameters.vertex.position)
    return;

  const auto &coords = m_actor.dataset.GetCoordinateSystem();
  const auto &cells = m_actor.dataset.GetCellSet();

  vtkm::Bounds coordBounds = coords.GetBounds();
  // set a default radius
  vtkm::Float64 lx = coordBounds.X.Length();
  vtkm::Float64 ly = coordBounds.Y.Length();
  vtkm::Float64 lz = coordBounds.Z.Length();
  vtkm::Float64 mag = vtkm::Sqrt(lx * lx + ly * ly + lz * lz);
  // same as used in vtk ospray
  constexpr vtkm::Float64 heuristic = 500.;
  auto baseRadius = static_cast<vtkm::Float32>(mag / heuristic);
  printf("baseRadius: %f\n", baseRadius);

  vtkm::rendering::raytracing::SphereExtractor sphereExtractor;

  sphereExtractor.ExtractCoordinates(coords, baseRadius);

  auto numPoints = sphereExtractor.GetNumberOfSpheres();
  m_parameters.numPrimitives = static_cast<uint32_t>(numPoints);

  if (numPoints == 0)
    printf("NO POINTS GENERATED\n");
  else {
    m_vertices = unpackPoints(sphereExtractor.GetPointIds(), coords);
    vtkm::cont::Token t;
    auto *p = (glm::vec3 *)m_vertices.GetBuffers()->ReadPointerHost(t);
    m_parameters.vertex.position = anari::newArray1D(m_device, p, numPoints);

    m_radii = sphereExtractor.GetRadii();
    auto *r = (float *)m_radii.GetBuffers()->ReadPointerHost(t);
    m_parameters.vertex.radius = anari::newArray1D(m_device, r, numPoints);
  }
}

} // namespace vtkm_anari
