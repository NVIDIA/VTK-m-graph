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

#include "ANARIMapperTriangles.h"
#include "vtkm/TriangleExtractor.h"
// anari + glm
#include <anari/anari_cpp/ext/glm.h>
// vtk-m
#include <vtkm/filter/vector_analysis/SurfaceNormals.h>
#include <vtkm/worklet/WorkletMapField.h>
// std
#include <numeric>

namespace vtkm_anari {

// Worklets ///////////////////////////////////////////////////////////////////

class ExtractTriangleVerticesAndNormals : public vtkm::worklet::WorkletMapField
{
 public:
  bool ExtractNormals{false};
  bool PopulateField{false};

  VTKM_CONT
  ExtractTriangleVerticesAndNormals(bool withNormals, bool emptyField)
      : ExtractNormals(withNormals), PopulateField(!emptyField)
  {}

  using ControlSignature = void(FieldIn,
      WholeArrayIn,
      WholeArrayIn,
      WholeArrayIn,
      WholeArrayOut,
      WholeArrayOut,
      WholeArrayOut);
  using ExecutionSignature = void(InputIndex, _1, _2, _3, _4, _5, _6, _7);

  template <typename PointPortalType,
      typename FieldPortalType,
      typename NormalPortalType,
      typename OutPointsPortalType,
      typename OutFieldPortalType,
      typename OutNormalsPortalType>
  VTKM_EXEC void operator()(const vtkm::Id idx,
      const vtkm::Id4 indices,
      const PointPortalType &points,
      const FieldPortalType &fields,
      const NormalPortalType &normals,
      OutPointsPortalType &outP,
      OutFieldPortalType &outF,
      OutNormalsPortalType &outN) const
  {
    auto i0 = indices[1];
    auto i1 = indices[2];
    auto i2 = indices[3];
    outP.Set(3 * idx + 0, static_cast<vtkm::Vec3f_32>(points.Get(i0)));
    outP.Set(3 * idx + 1, static_cast<vtkm::Vec3f_32>(points.Get(i1)));
    outP.Set(3 * idx + 2, static_cast<vtkm::Vec3f_32>(points.Get(i2)));
    if (this->PopulateField) {
      outF.Set(3 * idx + 0, static_cast<vtkm::Float32>(fields.Get(i0)));
      outF.Set(3 * idx + 1, static_cast<vtkm::Float32>(fields.Get(i1)));
      outF.Set(3 * idx + 2, static_cast<vtkm::Float32>(fields.Get(i2)));
    }
    if (this->ExtractNormals) {
      outN.Set(3 * idx + 0, static_cast<vtkm::Vec3f_32>(normals.Get(i0)));
      outN.Set(3 * idx + 1, static_cast<vtkm::Vec3f_32>(normals.Get(i1)));
      outN.Set(3 * idx + 2, static_cast<vtkm::Vec3f_32>(normals.Get(i2)));
    }
  }
};

// Helper functions ///////////////////////////////////////////////////////////

static TriangleArrays unpackTriangles(vtkm::cont::ArrayHandle<vtkm::Id4> tris,
    vtkm::cont::CoordinateSystem coords,
    vtkm::cont::ArrayHandle<vtkm::Float32> field,
    vtkm::cont::ArrayHandle<vtkm::Vec3f_32> normals)
{
  const auto numTris = tris.GetNumberOfValues();

  const bool emptyField = field.GetNumberOfValues() == 0;

  TriangleArrays retval;

  bool extractNormals = normals.GetNumberOfValues() != 0;

  retval.vertices.Allocate(numTris * 3);
  if (!emptyField)
    retval.attribute.Allocate(numTris * 3);
  if (extractNormals)
    retval.normals.Allocate(numTris * 3);

  ExtractTriangleVerticesAndNormals worklet(extractNormals, emptyField);
  vtkm::worklet::DispatcherMapField<ExtractTriangleVerticesAndNormals>(worklet)
      .Invoke(tris,
          coords,
          field,
          normals,
          retval.vertices,
          retval.attribute,
          retval.normals);

  return retval;
}

// ANARIMapperTriangles definitions ///////////////////////////////////////////

ANARIMapperTriangles::ANARIMapperTriangles(anari::Device device,
    const ANARIActor &actor,
    const char *name,
    const ColorTable &colorTable)
    : ANARIMapper(device, actor, name, colorTable)
{
  m_handles = std::make_shared<ANARIMapperTriangles::ANARIHandles>();
  m_handles->device = device;
  anari::retain(device, device);
}

void ANARIMapperTriangles::SetActor(const ANARIActor &actor)
{
  ANARIMapper::SetActor(actor);
  constructArrays(true);
}

void ANARIMapperTriangles::SetMapFieldAsAttribute(bool enabled)
{
  ANARIMapper::SetMapFieldAsAttribute(enabled);
  updateGeometry();
  updateMaterial();
}

void ANARIMapperTriangles::SetANARIColorMapArrays(anari::Array1D color,
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

void ANARIMapperTriangles::SetANARIColorMapValueRange(
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

const TrianglesParameters &ANARIMapperTriangles::Parameters()
{
  constructArrays();
  return m_handles->parameters;
}

void ANARIMapperTriangles::SetCalculateNormals(bool enabled)
{
  m_calculateNormals = enabled;
}

anari::Geometry ANARIMapperTriangles::GetANARIGeometry()
{
  if (m_handles->geometry)
    return m_handles->geometry;

  constructArrays();
  if (!m_handles->parameters.vertex.position)
    return nullptr;

  auto d = GetDevice();
  m_handles->geometry = anari::newObject<anari::Geometry>(d, "triangle");
  anari::setParameter(
      d, m_handles->geometry, "name", makeObjectName("geometry"));
  updateGeometry();
  return m_handles->geometry;
}

anari::Surface ANARIMapperTriangles::GetANARISurface()
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
  if (anari::deviceImplements(d, "VISRTX_SAMPLER_COLOR_MAP")) {
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

bool ANARIMapperTriangles::needToGenerateData() const
{
  const bool haveNormals = m_handles->parameters.vertex.normal != nullptr;
  const bool needNormals = m_calculateNormals && !haveNormals;
  return !m_current || needNormals;
}

void ANARIMapperTriangles::constructArrays(bool regenerate)
{
  if (regenerate)
    m_current = false;

  if (!regenerate && !needToGenerateData())
    return;

  m_current = true;
  m_valid = false;

  auto d = GetDevice();
  anari::release(d, m_handles->parameters.vertex.position);
  anari::release(d, m_handles->parameters.vertex.normal);
  anari::release(d, m_handles->parameters.vertex.attribute);
  anari::release(d, m_handles->parameters.primitive.index);
  m_handles->parameters.vertex.position = nullptr;
  m_handles->parameters.vertex.normal = nullptr;
  m_handles->parameters.vertex.attribute = nullptr;
  m_handles->parameters.primitive.index = nullptr;

  const auto &actor = GetActor();
  const auto &field = actor.GetField();
  const auto &cells = actor.GetCellSet();

  if (cells.GetNumberOfCells() == 0) {
    refreshGroup();
    return;
  }

  vtkm::rendering::raytracing::TriangleExtractor triExtractor;
  triExtractor.ExtractCells(cells);

  if (triExtractor.GetNumberOfTriangles() == 0) {
    refreshGroup();
    return;
  }

  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> inNormals;

  if (m_calculateNormals) {
    vtkm::filter::vector_analysis::SurfaceNormals normalsFilter;
    normalsFilter.SetOutputFieldName("Normals");
    auto dataset = normalsFilter.Execute(actor.MakeDataSet());
    auto field = dataset.GetField("Normals");
    auto fieldArray = field.GetData();
    inNormals = fieldArray.AsArrayHandle<decltype(m_arrays.normals)>();
  }

  const bool emptyField = field.GetNumberOfValues() == 0;

  using AttributeHandleT = decltype(m_arrays.attribute);
  auto arrays = unpackTriangles(triExtractor.GetTriangles(),
      actor.GetCoordinateSystem(),
      emptyField ? AttributeHandleT{}
                 : field.GetData().AsArrayHandle<AttributeHandleT>(),
      inNormals);

  auto numVerts = arrays.vertices.GetNumberOfValues();

  auto *v =
      (glm::vec3 *)arrays.vertices.GetBuffers()->ReadPointerHost(*arrays.token);
  auto *a = emptyField
      ? nullptr
      : (float *)arrays.attribute.GetBuffers()->ReadPointerHost(*arrays.token);
  auto *n =
      (glm::vec3 *)arrays.normals.GetBuffers()->ReadPointerHost(*arrays.token);

  m_handles->parameters.numPrimitives = numVerts / 3;
  m_handles->parameters.vertex.position =
      anari::newArray1D(d, v, noopANARIDeleter, nullptr, numVerts);
  if (a) {
    m_handles->parameters.vertex.attribute =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numVerts);
  }

  if (m_calculateNormals) {
    m_handles->parameters.vertex.normal = anari::newArray1D(
        d, n, noopANARIDeleter, nullptr, arrays.normals.GetNumberOfValues());
  }

  // NOTE: usd device requires indices, but shouldn't
  {
    auto indexArray = anari::newArray1D(
        d, ANARI_UINT32_VEC3, m_handles->parameters.numPrimitives);
    auto *begin = anari::map<unsigned int>(d, indexArray);
    auto *end = begin + numVerts;
    std::iota(begin, end, 0);
    anari::unmap(d, indexArray);
    m_handles->parameters.primitive.index = indexArray;
  }

  updateGeometry();

  m_arrays = arrays;
  m_valid = true;

  refreshGroup();
}

void ANARIMapperTriangles::updateGeometry()
{
  if (!m_handles->geometry)
    return;
  auto d = GetDevice();
  anari::setParameter(d,
      m_handles->geometry,
      "vertex.position",
      m_handles->parameters.vertex.position);
  if (GetMapFieldAsAttribute()) {
    anari::setParameter(d,
        m_handles->geometry,
        "vertex.attribute0",
        m_handles->parameters.vertex.attribute);
  } else {
    anari::unsetParameter(d, m_handles->geometry, "vertex.attribute0");
  }
  if (m_calculateNormals) {
    anari::setParameter(d,
        m_handles->geometry,
        "vertex.normal",
        m_handles->parameters.vertex.normal);
  } else {
    anari::unsetParameter(d, m_handles->geometry, "vertex.normal");
  }
  anari::setParameter(d,
      m_handles->geometry,
      "primitive.index",
      m_handles->parameters.primitive.index);
  anari::commitParameters(d, m_handles->geometry);
}

void ANARIMapperTriangles::updateMaterial()
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

ANARIMapperTriangles::ANARIHandles::~ANARIHandles()
{
  anari::release(device, surface);
  anari::release(device, material);
  anari::release(device, sampler);
  anari::release(device, geometry);
  anari::release(device, parameters.vertex.position);
  anari::release(device, parameters.vertex.normal);
  anari::release(device, parameters.vertex.attribute);
  anari::release(device, parameters.primitive.index);
  anari::release(device, device);
}

} // namespace vtkm_anari
