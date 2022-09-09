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
#include "NormalizedFieldValue.h"
#include "vtkm/SphereExtractor.h"
// anari + glm
#include <anari/anari_cpp/ext/glm.h>
// vtk-m
#include <vtkm/worklet/WorkletMapField.h>

#define USE_TEXTURES 1

namespace vtkm_anari {

// Worklets ///////////////////////////////////////////////////////////////////

class ExtractPointFields : public vtkm::worklet::WorkletMapField
{
 public:
  bool PopulateField1{false};
  bool PopulateField2{false};
  bool PopulateField3{false};
  bool PopulateField4{false};
  vtkm::Range Field1Range;
  vtkm::Range Field2Range;
  vtkm::Range Field3Range;
  vtkm::Range Field4Range;

  VTKM_CONT
  ExtractPointFields(bool emptyField1,
      bool emptyField2,
      bool emptyField3,
      bool emptyField4,
      vtkm::Range field1Range,
      vtkm::Range field2Range,
      vtkm::Range field3Range,
      vtkm::Range field4Range)
      : PopulateField1(!emptyField1),
        PopulateField2(!emptyField2),
        PopulateField3(!emptyField3),
        PopulateField4(!emptyField4),
        Field1Range(field1Range),
        Field2Range(field2Range),
        Field3Range(field3Range),
        Field4Range(field4Range)
  {}

  using ControlSignature = void(FieldIn, // [in] index
      WholeArrayIn, // [in] field1
      WholeArrayIn, // [in] field2
      WholeArrayIn, // [in] field3
      WholeArrayIn, // [in] field4
      WholeArrayOut, // [out] field1
      WholeArrayOut, // [out] field2
      WholeArrayOut, // [out] field3
      WholeArrayOut // [out] field4
  );
  using ExecutionSignature = void(InputIndex,
      _1, // [in] index
      _2, // [in] field1
      _3, // [in] field2
      _4, // [in] field3
      _5, // [in] field4
      _6, // [out] field1
      _7, // [out] field2
      _8, // [out] field3
      _9 // [out] field4
  );

  template <typename InFieldPortalType, typename OutFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::Id out_idx,
      const vtkm::Id in_idx,
      const InFieldPortalType &field1,
      const InFieldPortalType &field2,
      const InFieldPortalType &field3,
      const InFieldPortalType &field4,
      OutFieldPortalType &outF1,
      OutFieldPortalType &outF2,
      OutFieldPortalType &outF3,
      OutFieldPortalType &outF4) const
  {
    if (this->PopulateField1)
      outF1.Set(out_idx, NormalizedFieldValue(field1.Get(in_idx), Field1Range));
    if (this->PopulateField2)
      outF2.Set(out_idx, NormalizedFieldValue(field2.Get(in_idx), Field2Range));
    if (this->PopulateField3)
      outF3.Set(out_idx, NormalizedFieldValue(field3.Get(in_idx), Field3Range));
    if (this->PopulateField4)
      outF4.Set(out_idx, NormalizedFieldValue(field4.Get(in_idx), Field4Range));
  }
};

class ExtractPointPositions : public vtkm::worklet::WorkletMapField
{
 public:
  VTKM_CONT
  ExtractPointPositions() = default;

  using ControlSignature = void(FieldIn, // [in] index
      WholeArrayIn, // [in] point
      WholeArrayOut // [out] point
  );
  using ExecutionSignature = void(InputIndex,
      _1, // [in] index
      _2, // [in] point
      _3 // [out] points
  );

  template <typename InPointPortalType, typename OutPointPortalType>
  VTKM_EXEC void operator()(const vtkm::Id out_idx,
      const vtkm::Id in_idx,
      const InPointPortalType &points,
      OutPointPortalType &outP) const
  {
    outP.Set(out_idx, static_cast<vtkm::Vec3f_32>(points.Get(in_idx)));
  }
};

// Helper functions ///////////////////////////////////////////////////////////

static PointsFieldArrays unpackFields(
    vtkm::cont::ArrayHandle<vtkm::Id> points, FieldSet fields)
{
  PointsFieldArrays retval;

  const auto numPoints = points.GetNumberOfValues();

  const bool emptyField1 = fields[0].GetNumberOfValues() == 0;
  const bool emptyField2 = fields[1].GetNumberOfValues() == 0;
  const bool emptyField3 = fields[2].GetNumberOfValues() == 0;
  const bool emptyField4 = fields[3].GetNumberOfValues() == 0;

  using AttributeHandleT = decltype(retval.field1);

  auto field1 = emptyField1
      ? AttributeHandleT{}
      : fields[0].GetData().AsArrayHandle<AttributeHandleT>();
  auto field2 = emptyField2
      ? AttributeHandleT{}
      : fields[1].GetData().AsArrayHandle<AttributeHandleT>();
  auto field3 = emptyField3
      ? AttributeHandleT{}
      : fields[2].GetData().AsArrayHandle<AttributeHandleT>();
  auto field4 = emptyField4
      ? AttributeHandleT{}
      : fields[3].GetData().AsArrayHandle<AttributeHandleT>();

  vtkm::Range field1Range;
  vtkm::Range field2Range;
  vtkm::Range field3Range;
  vtkm::Range field4Range;

  if (!emptyField1) {
    retval.field1.Allocate(numPoints);
    fields[0].GetRange(&field1Range);
  }
  if (!emptyField2) {
    retval.field2.Allocate(numPoints);
    fields[1].GetRange(&field2Range);
  }
  if (!emptyField3) {
    retval.field3.Allocate(numPoints);
    fields[2].GetRange(&field3Range);
  }
  if (!emptyField4) {
    retval.field4.Allocate(numPoints);
    fields[3].GetRange(&field4Range);
  }

  ExtractPointFields fieldsWorklet(emptyField1,
      emptyField2,
      emptyField3,
      emptyField4,
      field1Range,
      field2Range,
      field3Range,
      field4Range);
  vtkm::worklet::DispatcherMapField<ExtractPointFields>(fieldsWorklet)
      .Invoke(points,
          field1,
          field2,
          field3,
          field4,
          retval.field1,
          retval.field2,
          retval.field3,
          retval.field4);

  return retval;
}

static PointsArrays unpackPoints(vtkm::cont::ArrayHandle<vtkm::Id> points,
    vtkm::cont::CoordinateSystem coords)
{
  PointsArrays retval;

  const auto numPoints = points.GetNumberOfValues();
  retval.vertices.Allocate(numPoints);
  vtkm::worklet::DispatcherMapField<ExtractPointPositions>().Invoke(
      points, coords, retval.vertices);

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
  auto &attributes = m_handles->parameters.vertex.attribute;
  std::fill(attributes.begin(), attributes.end(), nullptr);
  anari::retain(device, device);
}

void ANARIMapperPoints::SetActor(const ANARIActor &actor)
{
  ANARIMapper::SetActor(actor);
  constructArrays(true);
  updateMaterial();
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
#if USE_TEXTURES
    anari::setParameter(d, s, "image", color);
#else
    anari::setParameter(d, s, "color", color);
    anari::setParameter(d, s, "color.position", color_position);
#endif
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
  constructArrays();
  return m_handles->parameters;
}

anari::Geometry ANARIMapperPoints::GetANARIGeometry()
{
  constructArrays();
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

  bool isVisRTX = false;
  anari::getProperty(d, d, "visrtx", isVisRTX);
  if (isVisRTX) {
#if USE_TEXTURES
    auto s = anari::newObject<anari::Sampler>(d, "image1D");
    m_handles->sampler = s;
    auto colorArray = anari::newArray1D(d, ANARI_FLOAT32_VEC3, 3);
    auto *colors = anari::map<glm::vec3>(d, colorArray);
    colors[0] = glm::vec3(1.f, 0.f, 0.f);
    colors[1] = glm::vec3(0.f, 1.f, 0.f);
    colors[2] = glm::vec3(0.f, 0.f, 1.f);
    anari::unmap(d, colorArray);
    anari::setAndReleaseParameter(d, s, "image", colorArray);
    anari::setParameter(d, s, "name", makeObjectName("colormap"));
    anari::setParameter(d, s, "filter", "linear");
    anari::setParameter(d, s, "wrapMode1", "clampToEdge");
    anari::commitParameters(d, s);
#else
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
    anari::setParameter(d, s, "name", makeObjectName("colormap"));
    anari::commitParameters(d, s);
#endif
  }

  updateMaterial();

  m_handles->surface = anari::newObject<anari::Surface>(d);
  anari::setParameter(d, m_handles->surface, "name", makeObjectName("surface"));
  anari::setParameter(d, m_handles->surface, "geometry", geometry);
  anari::setParameter(d, m_handles->surface, "material", m_handles->material);
  anari::commitParameters(d, m_handles->surface);

  return m_handles->surface;
}

void ANARIMapperPoints::constructArrays(bool regenerate)
{
  if (regenerate)
    m_current = false;

  if (m_current)
    return;

  m_current = true;
  m_valid = false;

  m_handles->releaseArrays();

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
  m_handles->parameters.numPrimitives = static_cast<uint32_t>(numPoints);

  if (numPoints == 0) {
    refreshGroup();
    return;
  }

  m_primaryField = actor.GetPrimaryField();

  auto pts = sphereExtractor.GetPointIds();

  auto arrays = unpackPoints(pts, coords);
  auto fieldArrays = unpackFields(pts, actor.GetFieldSet());

  arrays.radii = sphereExtractor.GetRadii();
  auto *p =
      (glm::vec3 *)arrays.vertices.GetBuffers()->ReadPointerHost(*arrays.token);
  auto *r = (float *)arrays.radii.GetBuffers()->ReadPointerHost(*arrays.token);

  auto d = GetDevice();
  m_handles->parameters.vertex.position =
      anari::newArray1D(d, p, noopANARIDeleter, nullptr, numPoints);
  m_handles->parameters.vertex.radius =
      anari::newArray1D(d, r, noopANARIDeleter, nullptr, numPoints);

  if (fieldArrays.field1.GetNumberOfValues() != 0) {
    auto *a = (float *)fieldArrays.field1.GetBuffers()->ReadPointerHost(
        *fieldArrays.token);
    m_handles->parameters.vertex.attribute[0] =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numPoints);
  }
  if (fieldArrays.field2.GetNumberOfValues() != 0) {
    auto *a = (float *)fieldArrays.field2.GetBuffers()->ReadPointerHost(
        *fieldArrays.token);
    m_handles->parameters.vertex.attribute[1] =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numPoints);
  }
  if (fieldArrays.field3.GetNumberOfValues() != 0) {
    auto *a = (float *)fieldArrays.field3.GetBuffers()->ReadPointerHost(
        *fieldArrays.token);
    m_handles->parameters.vertex.attribute[2] =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numPoints);
  }
  if (fieldArrays.field4.GetNumberOfValues() != 0) {
    auto *a = (float *)fieldArrays.field4.GetBuffers()->ReadPointerHost(
        *fieldArrays.token);
    m_handles->parameters.vertex.attribute[3] =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numPoints);
  }

  updateGeometry();

  m_arrays = arrays;
  m_fieldArrays = fieldArrays;
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
        m_handles->parameters.vertex.attribute[0]);
    anari::setParameter(d,
        m_handles->geometry,
        "vertex.attribute1",
        m_handles->parameters.vertex.attribute[1]);
    anari::setParameter(d,
        m_handles->geometry,
        "vertex.attribute2",
        m_handles->parameters.vertex.attribute[2]);
    anari::setParameter(d,
        m_handles->geometry,
        "vertex.attribute3",
        m_handles->parameters.vertex.attribute[3]);
  } else {
    anari::unsetParameter(d, m_handles->geometry, "vertex.attribute0");
    anari::unsetParameter(d, m_handles->geometry, "vertex.attribute1");
    anari::unsetParameter(d, m_handles->geometry, "vertex.attribute2");
    anari::unsetParameter(d, m_handles->geometry, "vertex.attribute3");
  }
  anari::commitParameters(d, m_handles->geometry);
}

void ANARIMapperPoints::updateMaterial()
{
  if (!m_handles->material)
    return;

  auto d = GetDevice();
  auto s = m_handles->sampler;
  auto a = m_handles->parameters.vertex.attribute[m_primaryField];
  if (s && a && GetMapFieldAsAttribute()) {
    anari::setParameter(
        d, s, "inAttribute", anariMaterialInputString(m_primaryField));
    anari::commitParameters(d, s);
    anari::setParameter(d, m_handles->material, "color", s);
  } else
    anari::setParameter(d, m_handles->material, "color", glm::vec3(1.f));

  anari::commitParameters(d, m_handles->material);
}

ANARIMapperPoints::ANARIHandles::~ANARIHandles()
{
  releaseArrays();
  anari::release(device, surface);
  anari::release(device, material);
  anari::release(device, sampler);
  anari::release(device, geometry);
  anari::release(device, device);
}

void ANARIMapperPoints::ANARIHandles::releaseArrays()
{
  anari::release(device, parameters.vertex.position);
  anari::release(device, parameters.vertex.radius);
  anari::release(device, parameters.vertex.attribute[0]);
  anari::release(device, parameters.vertex.attribute[1]);
  anari::release(device, parameters.vertex.attribute[2]);
  anari::release(device, parameters.vertex.attribute[3]);
  parameters.vertex.position = nullptr;
  parameters.vertex.radius = nullptr;
  parameters.vertex.attribute[0] = nullptr;
  parameters.vertex.attribute[1] = nullptr;
  parameters.vertex.attribute[2] = nullptr;
  parameters.vertex.attribute[3] = nullptr;
}

} // namespace vtkm_anari
