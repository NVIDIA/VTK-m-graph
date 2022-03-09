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

#include "ANARIMapper.h"
#include "ExtractTriangleVertices.h"
// vtk-m
#include <vtkm/rendering/raytracing/TriangleExtractor.h>
#include <vtkm/source/Tangle.h>
// anari
#include <anari/anari_cpp/ext/glm.h>
// std
#include <numeric>

namespace vtkm_anari {

// FieldIndex definitions /////////////////////////////////////////////////////

FieldIndex::FieldIndex(vtkm::Id i) : type(FieldIndexType::ID), id(i) {}

FieldIndex::FieldIndex(std::string n) : type(FieldIndexType::STRING), name(n) {}

// Helper functions ///////////////////////////////////////////////////////////

static renderable::Object makeVolume(anari::Device d,
    const vtkm::cont::DataSet &dataset,
    const vtkm::cont::Field &dataField)
{
  renderable::Object retval;

  const auto &coords = dataset.GetCoordinateSystem();
  const auto &cells = dataset.GetCellSet();
  const auto &fieldArray = dataField.GetData();

  if (!cells.IsType<vtkm::cont::CellSetStructured<3>>())
    printf("CELLS ARE NOT STRUCTURED\n");
  else if (fieldArray
               .CanConvert<vtkm::cont::ArrayHandleConstant<vtkm::Float32>>())
    printf("FIELD DATA NOT FLOAT32\n");
  else {
    retval.device = d;

    auto structuredCells = cells.AsCellSet<vtkm::cont::CellSetStructured<3>>();
    auto pdims = structuredCells.GetPointDimensions();
    auto pointAH =
        fieldArray.AsArrayHandle<vtkm::cont::ArrayHandle<vtkm::Float32>>();

    vtkm::cont::Token t;
    auto *ptr = (float *)pointAH.GetBuffers()->ReadPointerHost(t);

    auto bounds = coords.GetBounds();
    glm::vec3 bLower(bounds.X.Min, bounds.Y.Min, bounds.Z.Min);
    glm::vec3 bUpper(bounds.X.Max, bounds.Y.Max, bounds.Z.Max);
    glm::vec3 size = bUpper - bLower;

    glm::uvec3 dims(pdims[0], pdims[1], pdims[2]);
    auto spacing = size / (glm::vec3(dims) - 1.f);

    std::memcpy(retval.object.volume.dims, &dims, sizeof(dims));
    std::memcpy(retval.object.volume.origin, &bLower, sizeof(bLower));
    std::memcpy(retval.object.volume.spacing, &spacing, sizeof(spacing));
    retval.object.volume.data =
        anari::newArray3D(d, ptr, dims.x, dims.y, dims.z);
    retval.type = RenderableObjectType::VOLUME;
  }

  return retval;
}

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

static renderable::Object makeTriangles(
    anari::Device d, const vtkm::cont::DataSet &dataset)
{
  renderable::Object retval;

  const auto &cells = dataset.GetCellSet();

  vtkm::rendering::raytracing::TriangleExtractor triExtractor;
  triExtractor.ExtractCells(cells);

  const auto numTriangles = triExtractor.GetNumberOfTriangles();

  if (numTriangles == 0)
    printf("NO TRIANGLES GENERATED\n");
  else {
    retval.device = d;

    auto vertices = unpackTriangleVertices(
        triExtractor.GetTriangles(), dataset.GetCoordinateSystem());
    auto numVerts = vertices.GetNumberOfValues();

    vtkm::cont::Token t;
    auto *v = (glm::vec3 *)vertices.GetBuffers()->ReadPointerHost(t);

    retval.object.triangles.vertex.position = anari::newArray1D(d, v, numVerts);
#if 1 // NOTE: usd device requires indices, but shouldn't
    {
      auto indexArray = anari::newArray1D(d, ANARI_UINT32_VEC3, numVerts / 3);
      auto *begin = (unsigned int *)anari::map(d, indexArray);
      auto *end = begin + numVerts;
      std::iota(begin, end, 0);
      anari::unmap(d, indexArray);
      retval.object.triangles.primitive.index = indexArray;
    }
#endif
    retval.type = RenderableObjectType::TRIANGLES;
  }

  return retval;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

renderable::Object makeANARIObject(anari::Device d, Actor actor)
{
  const vtkm::cont::Field *field = nullptr;

  if (actor.fieldIndex.type == FieldIndexType::ID)
    field = &actor.dataset.GetField(actor.fieldIndex.id);
  else
    field = &actor.dataset.GetField(actor.fieldIndex.name);

  if (actor.representation == Representation::VOLUME)
    return makeVolume(d, actor.dataset, *field);
  else
    return makeTriangles(d, actor.dataset);
}

static void releaseArrays(anari::Device d, renderable::Triangles t)
{
  anari::release(d, t.vertex.position);
  anari::release(d, t.vertex.attribute);
  anari::release(d, t.primitive.index);
  anari::release(d, t.primitive.attribute);
}

static void releaseArrays(anari::Device d, renderable::Volume v)
{
  anari::release(d, v.data);
}

void releaseHandles(renderable::Object o)
{
  auto d = o.device;
  if (!d)
    return;

  if (o.type == RenderableObjectType::TRIANGLES)
    releaseArrays(d, o.object.triangles);
  else if (o.type == RenderableObjectType::VOLUME)
    releaseArrays(d, o.object.volume);
}

} // namespace vtkm_anari
