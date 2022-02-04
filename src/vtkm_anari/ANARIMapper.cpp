// Copyright 2022 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "ANARIMapper.h"
#include "ExtractTriangleVertices.h"
// vtk-m
#include <vtkm/rendering/raytracing/TriangleExtractor.h>
#include <vtkm/source/Tangle.h>
// anari
#include <anari/anari_cpp/ext/glm.h>
// std
#include <numeric>

namespace vtkm3D {

// Helper functions ///////////////////////////////////////////////////////////

static anari::Volume makeANARIVolume(anari::Device d,
    const vtkm::cont::DataSet &dataset,
    const vtkm::cont::Field &dataField)
{
  anari::Volume volume = {};

  const auto &coords = dataset.GetCoordinateSystem();
  const auto &cells = dataset.GetCellSet();
  const auto &fieldArray = dataField.GetData();

  if (!cells.IsType<vtkm::cont::CellSetStructured<3>>())
    printf("CELLS ARE NOT STRUCTURED\n");
  else if (fieldArray
               .CanConvert<vtkm::cont::ArrayHandleConstant<vtkm::Float32>>())
    printf("FIELD DATA NOT FLOAT32\n");
  else {
    auto structuredCells = cells.AsCellSet<vtkm::cont::CellSetStructured<3>>();
    auto pdims = structuredCells.GetPointDimensions();
    auto pointAH =
        fieldArray.AsArrayHandle<vtkm::cont::ArrayHandle<vtkm::Float32>>();

    vtkm::cont::Token t;
    auto *ptr = (float *)pointAH.GetBuffers()->ReadPointerHost(t);

    glm::uvec3 dims(pdims[0], pdims[1], pdims[2]);

    auto bounds = coords.GetBounds();
    glm::vec3 bLower(bounds.X.Min, bounds.Y.Min, bounds.Z.Min);
    glm::vec3 bUpper(bounds.X.Max, bounds.Y.Max, bounds.Z.Max);
    glm::vec3 size = bUpper - bLower;

    auto field = anari::newObject<anari::SpatialField>(d, "structuredRegular");
    anari::setParameter(d, field, "origin", bLower);
    anari::setParameter(d, field, "spacing", size / (glm::vec3(dims) - 1.f));
    anari::setAndReleaseParameter(
        d, field, "data", anari::newArray3D(d, ptr, dims.x, dims.y, dims.z));
    anari::commit(d, field);

    volume = anari::newObject<anari::Volume>(d, "scivis");
    anari::setAndReleaseParameter(d, volume, "field", field);
    anari::setParameter(d, volume, "densityScale", 0.05f);

    {
      std::vector<glm::vec3> colors;
      std::vector<float> opacities;

      colors.emplace_back(0.f, 0.f, 1.f);
      colors.emplace_back(0.f, 1.f, 0.f);
      colors.emplace_back(1.f, 0.f, 0.f);

      opacities.emplace_back(0.f);
      opacities.emplace_back(1.f);

      anari::setAndReleaseParameter(d,
          volume,
          "color",
          anari::newArray1D(d, colors.data(), colors.size()));
      anari::setAndReleaseParameter(d,
          volume,
          "opacity",
          anari::newArray1D(d, opacities.data(), opacities.size()));
      anari::setParameter(d, volume, "valueRange", glm::vec2(0.f, 10.f));
    }

    anari::commit(d, volume);
  }

  return volume;
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

static anari::Surface makeANARISurface(
    anari::Device d, const vtkm::cont::DataSet &dataset)
{
  anari::Surface surface = {};

  const auto &cells = dataset.GetCellSet();

  vtkm::rendering::raytracing::TriangleExtractor triExtractor;
  triExtractor.ExtractCells(cells);

  const auto numTriangles = triExtractor.GetNumberOfTriangles();

  if (numTriangles == 0)
    printf("NO TRIANGLES GENERATED\n");
  else {
    surface = anari::newObject<anari::Surface>(d);

    auto vertices = unpackTriangleVertices(
        triExtractor.GetTriangles(), dataset.GetCoordinateSystem());
    auto numVerts = vertices.GetNumberOfValues();

    vtkm::cont::Token t;
    auto *v = (glm::vec3 *)vertices.GetBuffers()->ReadPointerHost(t);

    auto geom = anari::newObject<anari::Geometry>(d, "triangle");
    anari::setAndReleaseParameter(
        d, geom, "vertex.position", anari::newArray1D(d, v, numVerts));

#if 1 // NOTE: usd device requires indices, but shouldn't
    std::vector<uint32_t> indices(numVerts);
    std::iota(indices.begin(), indices.end(), 0);
    anari::setAndReleaseParameter(d,
        geom,
        "primitive.index",
        anari::newArray1D(d, (glm::uvec3 *)indices.data(), indices.size() / 3));
#endif

    anari::commit(d, geom);

    anari::setAndReleaseParameter(d, surface, "geometry", geom);

    auto mat = anari::newObject<anari::Material>(d, "matte");
    anari::setParameter(d, mat, "color", glm::vec4(1.f));
    anari::commit(d, mat);
    anari::setAndReleaseParameter(d, surface, "material", mat);

    anari::commit(d, surface);
  }

  return surface;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

RenderableObject makeANARIObject(anari::Device d, Actor actor)
{
  RenderableObject retval;

  const vtkm::cont::Field *field = nullptr;

  if (std::holds_alternative<vtkm::Id>(actor.field))
    field = &actor.dataset.GetField(std::get<vtkm::Id>(actor.field));
  else
    field = &actor.dataset.GetField(std::get<std::string>(actor.field));

  if (actor.representation == Representation::VOLUME) {
    auto v = makeANARIVolume(d, actor.dataset, *field);
    if (v)
      retval = v;
  } else {
    auto s = makeANARISurface(d, actor.dataset);
    if (s)
      retval = s;
  }

  return retval;
}

} // namespace vtkm3D
