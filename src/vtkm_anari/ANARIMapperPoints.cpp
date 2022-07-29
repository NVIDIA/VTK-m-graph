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
  bool PopulateField{false};

  VTKM_CONT
  ExtractPointPositions(bool emptyField) : PopulateField(!emptyField) {}

  using ControlSignature = void(
      FieldIn, WholeArrayIn, WholeArrayIn, WholeArrayOut, WholeArrayOut);
  using ExecutionSignature = void(InputIndex, _1, _2, _3, _4, _5);

  template <typename InPointPortalType,
      typename InFieldPortalType,
      typename OutPointPortalType,
      typename OutFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::Id out_idx,
      const vtkm::Id in_idx,
      const InPointPortalType &points,
      const InFieldPortalType &fields,
      OutPointPortalType &outP,
      OutFieldPortalType &outF) const
  {
    outP.Set(out_idx, static_cast<vtkm::Vec3f_32>(points.Get(in_idx)));
    if (this->PopulateField)
      outF.Set(out_idx, static_cast<vtkm::Float32>(fields.Get(in_idx)));
  }
};

// Helper functions ///////////////////////////////////////////////////////////

static PointsArrays unpackPoints(vtkm::cont::ArrayHandle<vtkm::Id> points,
    vtkm::cont::ArrayHandle<vtkm::Float32> field,
    vtkm::cont::CoordinateSystem coords)
{
  const auto numPoints = points.GetNumberOfValues();

  const bool emptyField = field.GetNumberOfValues() == 0;

  PointsArrays retval;

  retval.vertices.Allocate(numPoints);
  if (!emptyField)
    retval.attribute.Allocate(numPoints);

  ExtractPointPositions worklet(emptyField);
  vtkm::worklet::DispatcherMapField<ExtractPointPositions>(worklet).Invoke(
      points, coords, field, retval.vertices, retval.attribute);

  return retval;
}

// ANARIMapperPoints definitions //////////////////////////////////////////////

ANARIMapperPoints::ANARIMapperPoints(anari::Device device,
    const ANARIActor &actor,
    const char *name,
    const ColorTable &colorTable)
    : ANARIMapper(device, actor, name, colorTable)
{
  m_handles = std::make_shared<ANARIMapperPoints::ANARIHandles>();
  m_handles->device = device;
  anari::retain(device, device);
}

void ANARIMapperPoints::SetActor(const ANARIActor &actor)
{
  ANARIMapper::SetActor(actor);
  constructParameters(true);
}

void ANARIMapperPoints::SetMapFieldAsAttribute(bool enabled)
{
  ANARIMapper::SetMapFieldAsAttribute(enabled);
  updateGeometry();
  updateMaterial();
}

void ANARIMapperPoints::SetANARIColorMapArrays(anari::Array1D color,
    anari::Array1D color_position,
    anari::Array1D opacity,
    anari::Array1D opacity_position,
    bool releaseArrays)
{
  GetANARISurface();
  auto s = m_handles->sampler;
  if (s) {
    auto d = GetDevice();
    anari::setParameter(d, s, "color", color);
    anari::setParameter(d, s, "color.position", color_position);
    anari::commitParameters(d, s);
  }
  ANARIMapper::SetANARIColorMapArrays(
      color, color_position, opacity, opacity_position, releaseArrays);
}

void ANARIMapperPoints::SetANARIColorMapValueRange(
    const vtkm::Vec2f_32 &valueRange)
{
  GetANARISurface();
  auto s = m_handles->sampler;
  if (s) {
    auto d = GetDevice();
    anari::setParameter(d, s, "valueRange", ANARI_FLOAT32_BOX1, &valueRange);
    anari::commitParameters(d, s);
  }
}

const PointsParameters &ANARIMapperPoints::Parameters()
{
  constructParameters();
  return m_handles->parameters;
}

anari::Geometry ANARIMapperPoints::GetANARIGeometry()
{
  constructParameters();
  if (!m_handles->parameters.vertex.position)
    return nullptr;

  if (m_handles->geometry)
    return m_handles->geometry;

  auto d = GetDevice();
  m_handles->geometry = anari::newObject<anari::Geometry>(d, "sphere");
  anari::setParameter(
      d, m_handles->geometry, "name", makeObjectName("geometry"));
  updateGeometry();
  return m_handles->geometry;
}

anari::Surface ANARIMapperPoints::GetANARISurface()
{
  auto geometry = GetANARIGeometry();
  if (!geometry)
    return nullptr;

  if (m_handles->surface)
    return m_handles->surface;

  auto d = GetDevice();

  if (!m_handles->material) {
    m_handles->material =
        anari::newObject<anari::Material>(d, "transparentMatte");
    anari::setParameter(
        d, m_handles->material, "name", makeObjectName("material"));
  }

#if 1
  if (false) {
#else
  if (!m_handles->sampler
      && anari::deviceImplements(d, "VISRTX_SAMPLER_COLOR_MAP")) {
#endif
    auto s = anari::newObject<anari::Sampler>(d, "colorMap");
    m_handles->sampler = s;
    auto colorArray = anari::newArray1D(d, ANARI_FLOAT32_VEC3, 3);
    auto *colors = anari::map<glm::vec3>(d, colorArray);
    colors[0] = glm::vec3(1.f, 0.f, 0.f);
    colors[1] = glm::vec3(0.f, 1.f, 0.f);
    colors[2] = glm::vec3(0.f, 0.f, 1.f);
    anari::unmap(d, colorArray);
    anari::setAndReleaseParameter(d, s, "color", colorArray);
    anari::setParameter(d, s, "valueRange", glm::vec2(0.f, 10.f));
    anari::setParameter(d, s, "inAttribute", "attribute0");
    anari::setParameter(d, s, "name", makeObjectName("colormap"));
    anari::commitParameters(d, s);
  }

  updateMaterial();

  m_handles->surface = anari::newObject<anari::Surface>(d);
  anari::setParameter(d, m_handles->surface, "name", makeObjectName("surface"));
  anari::setParameter(d, m_handles->surface, "geometry", geometry);
  anari::setParameter(d, m_handles->surface, "material", m_handles->material);
  anari::commitParameters(d, m_handles->surface);

  return m_handles->surface;
}

void ANARIMapperPoints::constructParameters(bool regenerate)
{
  if (!regenerate && m_handles->parameters.vertex.position)
    return;

  m_valid = false;

  auto d = GetDevice();
  anari::release(d, m_handles->parameters.vertex.position);
  anari::release(d, m_handles->parameters.vertex.radius);
  anari::release(d, m_handles->parameters.vertex.attribute);
  m_handles->parameters.vertex.position = nullptr;
  m_handles->parameters.vertex.radius = nullptr;
  m_handles->parameters.vertex.attribute = nullptr;

  const auto &actor = GetActor();
  const auto &coords = actor.GetCoordinateSystem();
  const auto &cells = actor.GetCellSet();
  const auto &field = actor.GetField();

  const bool emptyField = field.GetNumberOfValues() == 0;

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
  m_handles->parameters.numPrimitives = static_cast<uint32_t>(numPoints);

  if (numPoints == 0) {
    refreshGroup();
    return;
  }

  using AttributeHandleT = decltype(m_arrays.attribute);
  auto arrays = unpackPoints(sphereExtractor.GetPointIds(),
      emptyField ? AttributeHandleT{}
                 : field.GetData().AsArrayHandle<AttributeHandleT>(),
      coords);
  arrays.radii = sphereExtractor.GetRadii();
  auto *p =
      (glm::vec3 *)arrays.vertices.GetBuffers()->ReadPointerHost(*arrays.token);
  auto *r = (float *)arrays.radii.GetBuffers()->ReadPointerHost(*arrays.token);
  auto *a = emptyField
      ? nullptr
      : (float *)arrays.attribute.GetBuffers()->ReadPointerHost(*arrays.token);
  m_handles->parameters.vertex.position =
      anari::newArray1D(d, p, noopANARIDeleter, nullptr, numPoints);
  m_handles->parameters.vertex.radius =
      anari::newArray1D(d, r, noopANARIDeleter, nullptr, numPoints);
  if (a) {
    m_handles->parameters.vertex.attribute =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numPoints);
  }

  updateGeometry();

  m_arrays = arrays;
  m_valid = true;

  refreshGroup();
}

void ANARIMapperPoints::updateGeometry()
{
  if (!m_handles->geometry)
    return;
  auto d = GetDevice();
  anari::setParameter(d,
      m_handles->geometry,
      "vertex.position",
      m_handles->parameters.vertex.position);
  anari::setParameter(d,
      m_handles->geometry,
      "vertex.radius",
      m_handles->parameters.vertex.radius);
  if (GetMapFieldAsAttribute()) {
    anari::setParameter(d,
        m_handles->geometry,
        "vertex.attribute0",
        m_handles->parameters.vertex.attribute);
  } else {
    anari::unsetParameter(d, m_handles->geometry, "vertex.attribute0");
  }
  anari::commitParameters(d, m_handles->geometry);
}

void ANARIMapperPoints::updateMaterial()
{
  if (!m_handles->material)
    return;

  auto d = GetDevice();
  auto s = m_handles->sampler;
  auto a = m_handles->parameters.vertex.attribute;
  if (s && a && GetMapFieldAsAttribute())
    anari::setParameter(d, m_handles->material, "color", s);
  else
    anari::setParameter(d, m_handles->material, "color", glm::vec3(1.f));

  anari::commitParameters(d, m_handles->material);
}

ANARIMapperPoints::ANARIHandles::~ANARIHandles()
{
  anari::release(device, surface);
  anari::release(device, material);
  anari::release(device, sampler);
  anari::release(device, geometry);
  anari::release(device, parameters.vertex.position);
  anari::release(device, parameters.vertex.radius);
  anari::release(device, parameters.vertex.attribute);
  anari::release(device, device);
}

} // namespace vtkm_anari
