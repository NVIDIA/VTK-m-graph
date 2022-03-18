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

ANARIMapperTriangles::ANARIMapperTriangles(anari::Device device, Actor actor)
    : ANARIMapper(device, actor)
{}

ANARIMapperTriangles::~ANARIMapperTriangles()
{
  anari::release(m_device, m_parameters.vertex.position);
  anari::release(m_device, m_parameters.vertex.normal);
  anari::release(m_device, m_parameters.vertex.attribute);
  anari::release(m_device, m_parameters.primitive.index);
  anari::release(m_device, m_parameters.primitive.attribute);
}

const TrianglesParameters &ANARIMapperTriangles::Parameters()
{
  constructParameters();
  return m_parameters;
}

void ANARIMapperTriangles::SetCalculateNormals(bool enabled)
{
  m_calculateNormals = enabled;
}

anari::Geometry ANARIMapperTriangles::MakeGeometry()
{
  constructParameters();
  if (!m_parameters.vertex.position)
    return nullptr;
  auto geometry = anari::newObject<anari::Geometry>(m_device, "triangle");
  anari::setParameter(
      m_device, geometry, "vertex.position", m_parameters.vertex.position);
  if (m_calculateNormals) {
    anari::setParameter(
        m_device, geometry, "vertex.normal", m_parameters.vertex.normal);
  }
  anari::setParameter(
      m_device, geometry, "vertex.attribute0", m_parameters.vertex.attribute);
  anari::setParameter(
      m_device, geometry, "primitive.index", m_parameters.primitive.index);
  anari::setParameter(m_device,
      geometry,
      "primitive.attribute0",
      m_parameters.primitive.attribute);
  anari::commit(m_device, geometry);
  return geometry;
}

bool ANARIMapperTriangles::needToGenerateData() const
{
  const bool needPositions = m_parameters.vertex.position == nullptr;
  const bool haveNormals = m_parameters.vertex.normal != nullptr;
  const bool needNormals = m_calculateNormals && !haveNormals;
  return needPositions || needNormals;
}

void ANARIMapperTriangles::constructParameters()
{
  if (!needToGenerateData())
    return;

  auto &dataset = m_actor.dataset;

  vtkm::rendering::raytracing::TriangleExtractor triExtractor;
  triExtractor.ExtractCells(dataset.GetCellSet());

  if (triExtractor.GetNumberOfTriangles() == 0) {
    printf("NO TRIANGLES GENERATED\n");
    return;
  }

  vtkm::cont::Token t;

  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> inNormals;

  if (m_calculateNormals) {
    vtkm::filter::SurfaceNormals normalsFilter;
    normalsFilter.SetAutoOrientNormals(true);
    normalsFilter.SetFlipNormals(true);
    normalsFilter.SetOutputFieldName("Normals");
    dataset = normalsFilter.Execute(dataset);
    auto field = dataset.GetField("Normals");
    auto fieldArray = field.GetData();
    inNormals = fieldArray.AsArrayHandle<decltype(m_arrays.normals)>();
  }

  m_arrays = unpackTriangles(
      triExtractor.GetTriangles(), dataset.GetCoordinateSystem(), inNormals);

  auto numVerts = m_arrays.vertices.GetNumberOfValues();

  auto *v = (glm::vec3 *)m_arrays.vertices.GetBuffers()->ReadPointerHost(t);
  auto *n = (glm::vec3 *)m_arrays.normals.GetBuffers()->ReadPointerHost(t);

  m_parameters.numPrimitives = numVerts / 3;
  m_parameters.vertex.position = anari::newArray1D(m_device, v, numVerts);

  if (m_calculateNormals) {
    m_parameters.vertex.normal =
        anari::newArray1D(m_device, n, m_arrays.normals.GetNumberOfValues());
  }

  // NOTE: usd device requires indices, but shouldn't
  {
    auto indexArray = anari::newArray1D(
        m_device, ANARI_UINT32_VEC3, m_parameters.numPrimitives);
    auto *begin = (unsigned int *)anari::map(m_device, indexArray);
    auto *end = begin + numVerts;
    std::iota(begin, end, 0);
    anari::unmap(m_device, indexArray);
    m_parameters.primitive.index = indexArray;
  }
}

} // namespace vtkm_anari
