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
#include "detail/ExtractTriangleVertices.h"
// anari + glm
#include <anari/anari_cpp/ext/glm.h>
// vtk-m
#include <vtkm/rendering/raytracing/TriangleExtractor.h>
// std
#include <numeric>

namespace vtkm_anari {

// Helper functions ///////////////////////////////////////////////////////////

static vtkm::cont::ArrayHandle<vtkm::Vec3f_32> unpackTriangleVertices(
    vtkm::cont::ArrayHandle<vtkm::Id4> tris,
    vtkm::cont::CoordinateSystem coords)
{
  const auto numTris = tris.GetNumberOfValues();

  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> vertices;
  vertices.Allocate(numTris * 3);

  vtkm::worklet::DispatcherMapField<ExtractTriangleVertices>().Invoke(
      tris, coords, vertices);

  return vertices;
}

// ANARIMapperTriangles definitions ///////////////////////////////////////////

ANARIMapperTriangles::ANARIMapperTriangles(anari::Device device, Actor actor)
    : ANARIMapper(device, actor)
{}

ANARIMapperTriangles::~ANARIMapperTriangles()
{
  anari::release(m_device, m_parameters.vertex.position);
  anari::release(m_device, m_parameters.vertex.attribute);
  anari::release(m_device, m_parameters.primitive.index);
  anari::release(m_device, m_parameters.primitive.attribute);
}

const TrianglesParameters &ANARIMapperTriangles::parameters()
{
  constructParameters();
  return m_parameters;
}

anari::Geometry ANARIMapperTriangles::makeGeometry()
{
  constructParameters();
  auto geometry = anari::newObject<anari::Geometry>(m_device, "triangle");
  anari::setParameter(
      m_device, geometry, "vertex.position", m_parameters.vertex.position);
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

void ANARIMapperTriangles::constructParameters()
{
  if (m_parameters.vertex.position)
    return;

  const auto &cells = m_actor.dataset.GetCellSet();

  vtkm::rendering::raytracing::TriangleExtractor triExtractor;
  triExtractor.ExtractCells(cells);

  const auto numTriangles = triExtractor.GetNumberOfTriangles();

  if (numTriangles == 0)
    printf("NO TRIANGLES GENERATED\n");
  else {
    auto vertices = unpackTriangleVertices(
        triExtractor.GetTriangles(), m_actor.dataset.GetCoordinateSystem());
    auto numVerts = vertices.GetNumberOfValues();

    vtkm::cont::Token t;
    auto *v = (glm::vec3 *)vertices.GetBuffers()->ReadPointerHost(t);

    m_parameters.numPrimitives = numVerts / 3;
    m_parameters.vertex.position = anari::newArray1D(m_device, v, numVerts);

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
}

} // namespace vtkm_anari
