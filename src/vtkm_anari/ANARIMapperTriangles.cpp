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
#include <vtkm/filter/SurfaceNormals.h>
#include <vtkm/worklet/WorkletMapField.h>
// std
#include <numeric>

namespace vtkm_anari {

// Worklets ///////////////////////////////////////////////////////////////////

class ExtractTriangleVerticesAndNormals : public vtkm::worklet::WorkletMapField
{
 public:
  bool ExtractNormals{false};

  VTKM_CONT
  ExtractTriangleVerticesAndNormals(bool withNormals)
      : ExtractNormals(withNormals)
  {}

  using ControlSignature = void(
      FieldIn, WholeArrayIn, WholeArrayIn, WholeArrayOut, WholeArrayOut);
  using ExecutionSignature = void(InputIndex, _1, _2, _3, _4, _5);

  template <typename PointPortalType,
      typename NormalPortalType,
      typename OutPointsPortalType,
      typename OutNormalsPortalType>
  VTKM_EXEC void operator()(const vtkm::Id idx,
      const vtkm::Id4 indices,
      const PointPortalType &points,
      const NormalPortalType &normals,
      OutPointsPortalType &outP,
      OutNormalsPortalType &outN) const
  {
    auto i0 = indices[1];
    auto i1 = indices[2];
    auto i2 = indices[3];
    outP.Set(3 * idx + 0, static_cast<vtkm::Vec3f_32>(points.Get(i0)));
    outP.Set(3 * idx + 1, static_cast<vtkm::Vec3f_32>(points.Get(i1)));
    outP.Set(3 * idx + 2, static_cast<vtkm::Vec3f_32>(points.Get(i2)));
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
    vtkm::cont::ArrayHandle<vtkm::Vec3f_32> normals)
{
  const auto numTris = tris.GetNumberOfValues();

  TriangleArrays retval;

  bool extractNormals = normals.GetNumberOfValues() != 0;

  retval.vertices.Allocate(numTris * 3);
  if (extractNormals)
    retval.normals.Allocate(numTris * 3);

  ExtractTriangleVerticesAndNormals worklet(extractNormals);
  vtkm::worklet::DispatcherMapField<ExtractTriangleVerticesAndNormals>(worklet)
      .Invoke(tris, coords, normals, retval.vertices, retval.normals);

  return retval;
}

// ANARIMapperTriangles definitions ///////////////////////////////////////////

ANARIMapperTriangles::ANARIMapperTriangles(
    anari::Device device, const ANARIActor &actor, const ColorTable &colorTable)
    : ANARIMapper(device, actor, colorTable)
{
  m_handles = std::make_shared<ANARIMapperTriangles::ANARIHandles>();
  m_handles->device = device;
  anari::retain(device, device);
}

const TrianglesParameters &ANARIMapperTriangles::Parameters()
{
  constructParameters();
  return m_handles->parameters;
}

void ANARIMapperTriangles::SetCalculateNormals(bool enabled)
{
  m_calculateNormals = enabled;
}

anari::Geometry ANARIMapperTriangles::GetANARIGeometry()
{
  constructParameters();
  if (!m_handles->parameters.vertex.position)
    return nullptr;
  auto d = GetDevice();
  m_handles->geometry = anari::newObject<anari::Geometry>(d, "triangle");
  anari::setParameter(d,
      m_handles->geometry,
      "vertex.position",
      m_handles->parameters.vertex.position);
  if (m_calculateNormals) {
    anari::setParameter(d,
        m_handles->geometry,
        "vertex.normal",
        m_handles->parameters.vertex.normal);
  }
  anari::setParameter(d,
      m_handles->geometry,
      "vertex.attribute0",
      m_handles->parameters.vertex.attribute);
  anari::setParameter(d,
      m_handles->geometry,
      "primitive.index",
      m_handles->parameters.primitive.index);
  anari::setParameter(d,
      m_handles->geometry,
      "primitive.attribute0",
      m_handles->parameters.primitive.attribute);
  anari::commit(d, m_handles->geometry);
  return m_handles->geometry;
}

anari::Surface ANARIMapperTriangles::GetANARISurface()
{
  if (m_handles->surface)
    return m_handles->surface;

  auto geometry = GetANARIGeometry();
  if (!geometry)
    return nullptr;

  auto d = GetDevice();

  if (!m_handles->material) {
    m_handles->material =
        anari::newObject<anari::Material>(d, "transparentMatte");
  }

  m_handles->surface = anari::newObject<anari::Surface>(d);
  anari::setParameter(d, m_handles->surface, "geometry", geometry);
  anari::setParameter(d, m_handles->surface, "material", m_handles->material);
  anari::commit(d, m_handles->surface);

  return m_handles->surface;
}

bool ANARIMapperTriangles::needToGenerateData() const
{
  const bool needPositions = m_handles->parameters.vertex.position == nullptr;
  const bool haveNormals = m_handles->parameters.vertex.normal != nullptr;
  const bool needNormals = m_calculateNormals && !haveNormals;
  return needPositions || needNormals;
}

void ANARIMapperTriangles::constructParameters()
{
  if (!needToGenerateData())
    return;

  const auto &actor = GetActor();

  vtkm::rendering::raytracing::TriangleExtractor triExtractor;
  triExtractor.ExtractCells(actor.GetCellSet());

  if (triExtractor.GetNumberOfTriangles() == 0) {
    printf("NO TRIANGLES GENERATED\n");
    return;
  }

  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> inNormals;

  if (m_calculateNormals) {
    vtkm::filter::SurfaceNormals normalsFilter;
    normalsFilter.SetAutoOrientNormals(true);
    normalsFilter.SetFlipNormals(true);
    normalsFilter.SetOutputFieldName("Normals");
    auto dataset = normalsFilter.Execute(actor.MakeDataSet());
    auto field = dataset.GetField("Normals");
    auto fieldArray = field.GetData();
    inNormals = fieldArray.AsArrayHandle<decltype(m_arrays.normals)>();
  }

  m_arrays = unpackTriangles(
      triExtractor.GetTriangles(), actor.GetCoordinateSystem(), inNormals);

  auto numVerts = m_arrays.vertices.GetNumberOfValues();

  auto *v =
      (glm::vec3 *)m_arrays.vertices.GetBuffers()->ReadPointerHost(dataToken());
  auto *n =
      (glm::vec3 *)m_arrays.normals.GetBuffers()->ReadPointerHost(dataToken());

  auto d = GetDevice();
  m_handles->parameters.numPrimitives = numVerts / 3;
  m_handles->parameters.vertex.position =
      anari::newArray1D(d, v, noopANARIDeleter, nullptr, numVerts);

  if (m_calculateNormals) {
    m_handles->parameters.vertex.normal = anari::newArray1D(
        d, n, noopANARIDeleter, nullptr, m_arrays.normals.GetNumberOfValues());
  }

  // NOTE: usd device requires indices, but shouldn't
  {
    auto indexArray = anari::newArray1D(
        d, ANARI_UINT32_VEC3, m_handles->parameters.numPrimitives);
    auto *begin = (unsigned int *)anari::map(d, indexArray);
    auto *end = begin + numVerts;
    std::iota(begin, end, 0);
    anari::unmap(d, indexArray);
    m_handles->parameters.primitive.index = indexArray;
  }
}

ANARIMapperTriangles::ANARIHandles::~ANARIHandles()
{
  anari::release(device, surface);
  anari::release(device, material);
  anari::release(device, geometry);
  anari::release(device, parameters.vertex.position);
  anari::release(device, parameters.vertex.normal);
  anari::release(device, parameters.vertex.attribute);
  anari::release(device, parameters.primitive.index);
  anari::release(device, parameters.primitive.attribute);
  anari::release(device, device);
}

} // namespace vtkm_anari
