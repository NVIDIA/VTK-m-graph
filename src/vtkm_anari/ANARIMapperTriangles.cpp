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
  bool PopulateField1{false};
  bool PopulateField2{false};
  bool PopulateField3{false};
  bool PopulateField4{false};

  VTKM_CONT
  ExtractTriangleVerticesAndNormals(bool withNormals,
      bool emptyField1,
      bool emptyField2,
      bool emptyField3,
      bool emptyField4)
      : ExtractNormals(withNormals),
        PopulateField1(!emptyField1),
        PopulateField2(!emptyField2),
        PopulateField3(!emptyField3),
        PopulateField4(!emptyField4)
  {}

  using ControlSignature = void(FieldIn,
      WholeArrayIn, // [in] points
      WholeArrayIn, // [in] field1
      WholeArrayIn, // [in] field2
      WholeArrayIn, // [in] field3
      WholeArrayIn, // [in] field4
      WholeArrayIn, // [in] normals
      WholeArrayOut, // [out] points
      WholeArrayOut, // [out] field1
      WholeArrayOut, // [out] field2
      WholeArrayOut, // [out] field3
      WholeArrayOut, // [out] field4
      WholeArrayOut // [out] normals
  );
  using ExecutionSignature = void(InputIndex,
      _1, // [in] indices
      _2, // [in] points
      _3, // [in] field1
      _4, // [in] field2
      _5, // [in] field3
      _6, // [in] field4
      _7, // [in] normals
      _8, // [out] points
      _9, // [out] field1
      _10, // [out] field2
      _11, // [out] field3
      _12, // [out] field4
      _13 // [out] normals
  );

  template <typename PointPortalType,
      typename FieldPortalType,
      typename NormalPortalType,
      typename OutPointsPortalType,
      typename OutFieldPortalType,
      typename OutNormalsPortalType>
  VTKM_EXEC void operator()(const vtkm::Id idx,
      const vtkm::Id4 indices,
      const PointPortalType &points,
      const FieldPortalType &field1,
      const FieldPortalType &field2,
      const FieldPortalType &field3,
      const FieldPortalType &field4,
      const NormalPortalType &normals,
      OutPointsPortalType &outP,
      OutFieldPortalType &outF1,
      OutFieldPortalType &outF2,
      OutFieldPortalType &outF3,
      OutFieldPortalType &outF4,
      OutNormalsPortalType &outN) const
  {
    auto i0 = indices[1];
    auto i1 = indices[2];
    auto i2 = indices[3];
    outP.Set(3 * idx + 0, static_cast<vtkm::Vec3f_32>(points.Get(i0)));
    outP.Set(3 * idx + 1, static_cast<vtkm::Vec3f_32>(points.Get(i1)));
    outP.Set(3 * idx + 2, static_cast<vtkm::Vec3f_32>(points.Get(i2)));
    if (this->PopulateField1) {
      outF1.Set(3 * idx + 0, static_cast<vtkm::Float32>(field1.Get(i0)));
      outF1.Set(3 * idx + 1, static_cast<vtkm::Float32>(field1.Get(i1)));
      outF1.Set(3 * idx + 2, static_cast<vtkm::Float32>(field1.Get(i2)));
    }
    if (this->PopulateField2) {
      outF2.Set(3 * idx + 0, static_cast<vtkm::Float32>(field2.Get(i0)));
      outF2.Set(3 * idx + 1, static_cast<vtkm::Float32>(field2.Get(i1)));
      outF2.Set(3 * idx + 2, static_cast<vtkm::Float32>(field2.Get(i2)));
    }
    if (this->PopulateField3) {
      outF3.Set(3 * idx + 0, static_cast<vtkm::Float32>(field3.Get(i0)));
      outF3.Set(3 * idx + 1, static_cast<vtkm::Float32>(field3.Get(i1)));
      outF3.Set(3 * idx + 2, static_cast<vtkm::Float32>(field3.Get(i2)));
    }
    if (this->PopulateField4) {
      outF4.Set(3 * idx + 0, static_cast<vtkm::Float32>(field4.Get(i0)));
      outF4.Set(3 * idx + 1, static_cast<vtkm::Float32>(field4.Get(i1)));
      outF4.Set(3 * idx + 2, static_cast<vtkm::Float32>(field4.Get(i2)));
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
    FieldSet fields,
    vtkm::cont::ArrayHandle<vtkm::Vec3f_32> normals)
{
  TriangleArrays retval;

  const auto numTris = tris.GetNumberOfValues();

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

  bool extractNormals = normals.GetNumberOfValues() != 0;

  retval.vertices.Allocate(numTris * 3);
  if (!emptyField1)
    retval.field1.Allocate(numTris * 3);
  if (!emptyField2)
    retval.field2.Allocate(numTris * 3);
  if (!emptyField3)
    retval.field3.Allocate(numTris * 3);
  if (!emptyField4)
    retval.field4.Allocate(numTris * 3);
  if (extractNormals)
    retval.normals.Allocate(numTris * 3);

  ExtractTriangleVerticesAndNormals worklet(
      extractNormals, emptyField1, emptyField2, emptyField3, emptyField4);
  vtkm::worklet::DispatcherMapField<ExtractTriangleVerticesAndNormals>(worklet)
      .Invoke(tris,
          coords,
          field1,
          field2,
          field3,
          field4,
          normals,
          retval.vertices,
          retval.field1,
          retval.field2,
          retval.field3,
          retval.field4,
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
  auto &attributes = m_handles->parameters.vertex.attribute;
  std::fill(attributes.begin(), attributes.end(), nullptr);
  anari::retain(device, device);
}

void ANARIMapperTriangles::SetActor(const ANARIActor &actor)
{
  ANARIMapper::SetActor(actor);
  constructArrays(true);
  updateMaterial();
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
  bool isVisRTX = false;
  anari::getProperty(d, d, "visrtx", isVisRTX);
  if (isVisRTX) {
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
  anari::release(d, m_handles->parameters.vertex.attribute[0]);
  anari::release(d, m_handles->parameters.vertex.attribute[1]);
  anari::release(d, m_handles->parameters.vertex.attribute[2]);
  anari::release(d, m_handles->parameters.vertex.attribute[3]);
  anari::release(d, m_handles->parameters.primitive.index);
  m_handles->parameters.vertex.position = nullptr;
  m_handles->parameters.vertex.normal = nullptr;
  m_handles->parameters.vertex.attribute[0] = nullptr;
  m_handles->parameters.vertex.attribute[1] = nullptr;
  m_handles->parameters.vertex.attribute[2] = nullptr;
  m_handles->parameters.vertex.attribute[3] = nullptr;
  m_handles->parameters.primitive.index = nullptr;

  const auto &actor = GetActor();
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

  auto arrays = unpackTriangles(triExtractor.GetTriangles(),
      actor.GetCoordinateSystem(),
      actor.GetFieldSet(),
      inNormals);

  m_primaryField = actor.GetPrimaryField();

  auto numVerts = arrays.vertices.GetNumberOfValues();

  auto *v =
      (glm::vec3 *)arrays.vertices.GetBuffers()->ReadPointerHost(*arrays.token);
  auto *n =
      (glm::vec3 *)arrays.normals.GetBuffers()->ReadPointerHost(*arrays.token);

  m_handles->parameters.numPrimitives = numVerts / 3;
  m_handles->parameters.vertex.position =
      anari::newArray1D(d, v, noopANARIDeleter, nullptr, numVerts);

  if (arrays.field1.GetNumberOfValues() != 0) {
    auto *a =
        (float *)arrays.field1.GetBuffers()->ReadPointerHost(*arrays.token);
    m_handles->parameters.vertex.attribute[0] =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numVerts);
  }
  if (arrays.field2.GetNumberOfValues() != 0) {
    auto *a =
        (float *)arrays.field2.GetBuffers()->ReadPointerHost(*arrays.token);
    m_handles->parameters.vertex.attribute[1] =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numVerts);
  }
  if (arrays.field3.GetNumberOfValues() != 0) {
    auto *a =
        (float *)arrays.field3.GetBuffers()->ReadPointerHost(*arrays.token);
    m_handles->parameters.vertex.attribute[2] =
        anari::newArray1D(d, a, noopANARIDeleter, nullptr, numVerts);
  }
  if (arrays.field4.GetNumberOfValues() != 0) {
    auto *a =
        (float *)arrays.field4.GetBuffers()->ReadPointerHost(*arrays.token);
    m_handles->parameters.vertex.attribute[3] =
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
  auto a = m_handles->parameters.vertex.attribute[m_primaryField];
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
  anari::release(device, parameters.vertex.attribute[0]);
  anari::release(device, parameters.vertex.attribute[1]);
  anari::release(device, parameters.vertex.attribute[2]);
  anari::release(device, parameters.vertex.attribute[3]);
  anari::release(device, parameters.primitive.index);
  anari::release(device, device);
}

} // namespace vtkm_anari
