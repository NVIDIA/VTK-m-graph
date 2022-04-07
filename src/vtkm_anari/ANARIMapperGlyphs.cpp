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

#include "ANARIMapperGlyphs.h"
#include "vtkm/SphereExtractor.h"
// anari + glm
#include <anari/anari_cpp/ext/glm.h>
// vtk-m
#include <vtkm/VectorAnalysis.h>
#include <vtkm/filter/CellAverage.h>
#include <vtkm/worklet/WorkletMapField.h>

namespace vtkm_anari {

// Worklets ///////////////////////////////////////////////////////////////////

class GeneratePointGlyphs : public vtkm::worklet::WorkletMapField
{
 public:
  vtkm::Float32 SizeFactor{0.f};
  bool Offset{false};

  VTKM_CONT
  GeneratePointGlyphs(float size = 1.f, bool offset = false)
      : SizeFactor(size), Offset(offset)
  {}

  using ControlSignature = void(
      FieldIn, WholeArrayIn, WholeArrayOut, WholeArrayOut);
  using ExecutionSignature = void(InputIndex, _1, _2, _3, _4);

  template <typename InGradientType,
      typename InPointPortalType,
      typename OutVertexPortalType,
      typename OutRadiusPortalType>
  VTKM_EXEC void operator()(const vtkm::Id idx,
      const InGradientType gradient,
      const InPointPortalType &points,
      OutVertexPortalType &vertices,
      OutRadiusPortalType &radii) const
  {
    auto ng = vtkm::Normal(static_cast<vtkm::Vec3f_32>(gradient));
    auto pt = points.Get(idx);
    auto v0 = pt + ng * this->SizeFactor;
    auto v1 = pt + ng * -this->SizeFactor;
    if (this->Offset) {
      vertices.Set(4 * idx + 0, pt);
      vertices.Set(4 * idx + 1, v1);
      vertices.Set(4 * idx + 2, v1);
      vertices.Set(4 * idx + 3, v1 - (this->SizeFactor * ng));
    } else {
      vertices.Set(4 * idx + 0, v0);
      vertices.Set(4 * idx + 1, pt);
      vertices.Set(4 * idx + 2, pt);
      vertices.Set(4 * idx + 3, v1);
    }
    radii.Set(4 * idx + 0, this->SizeFactor / 8);
    radii.Set(4 * idx + 1, this->SizeFactor / 8);
    radii.Set(4 * idx + 2, this->SizeFactor / 4);
    radii.Set(4 * idx + 3, 0.f);
  }
};

// Helper functions ///////////////////////////////////////////////////////////

static GlyphArrays makeGlyphs(vtkm::cont::Field gradients,
    vtkm::cont::DynamicCellSet cells,
    vtkm::cont::CoordinateSystem coords,
    float glyphSize,
    bool offset)
{
  const auto numGlyphs = gradients.GetNumberOfValues();

  GlyphArrays retval;

  retval.vertices.Allocate(numGlyphs * 4);
  retval.radii.Allocate(numGlyphs * 4);

  GeneratePointGlyphs worklet(glyphSize, offset);
  vtkm::worklet::DispatcherMapField<GeneratePointGlyphs> dispatch(worklet);

  if (gradients.IsFieldPoint())
    dispatch.Invoke(gradients, coords, retval.vertices, retval.radii);
  else {
    vtkm::cont::DataSet centersInput;
    centersInput.AddCoordinateSystem(coords);
    centersInput.SetCellSet(cells);

    vtkm::filter::CellAverage filter;
    filter.SetUseCoordinateSystemAsField(true);
    filter.SetOutputFieldName("Centers");
    auto centersOutput = filter.Execute(centersInput);

    dispatch.Invoke(gradients,
        centersOutput.GetField("Centers"),
        retval.vertices,
        retval.radii);
  }

  return retval;
}

// ANARIMapperGlyphs definitions //////////////////////////////////////////////

ANARIMapperGlyphs::ANARIMapperGlyphs(anari::Device device,
    const ANARIActor &actor,
    const char *name,
    const ColorTable &colorTable)
    : ANARIMapper(device, actor, name, colorTable)
{
  m_handles = std::make_shared<ANARIMapperGlyphs::ANARIHandles>();
  m_handles->device = device;
  anari::retain(device, device);
}

void ANARIMapperGlyphs::SetActor(const ANARIActor &actor)
{
  ANARIMapper::SetActor(actor);
  constructParameters(true);
}

void ANARIMapperGlyphs::SetOffsetGlyphs(bool enabled)
{
  m_offset = enabled;
}

const GlyphsParameters &ANARIMapperGlyphs::Parameters()
{
  constructParameters();
  return m_handles->parameters;
}

anari::Geometry ANARIMapperGlyphs::GetANARIGeometry()
{
  if (m_handles->geometry)
    return m_handles->geometry;

  constructParameters();
  if (!m_handles->parameters.vertex.position)
    return nullptr;

  auto d = GetDevice();
  m_handles->geometry = anari::newObject<anari::Geometry>(d, "cone");
  updateGeometry();
  return m_handles->geometry;
}

anari::Surface ANARIMapperGlyphs::GetANARISurface()
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

  anari::commit(d, m_handles->material);

  m_handles->surface = anari::newObject<anari::Surface>(d);
  anari::setParameter(d, m_handles->surface, "geometry", geometry);
  anari::setParameter(d, m_handles->surface, "material", m_handles->material);
  anari::commit(d, m_handles->surface);
  return m_handles->surface;
}

void ANARIMapperGlyphs::constructParameters(bool regenerate)
{
  if (!regenerate && m_handles->parameters.vertex.position)
    return;

  const auto &actor = GetActor();
  const auto &coords = actor.GetCoordinateSystem();
  const auto &cells = actor.GetCellSet();
  const auto &field = actor.GetField();

  auto numGlyphs = field.GetNumberOfValues();

  if (numGlyphs == 0) {
    printf("NO GLYPHS GENERATED\n");
    return;
  }

  auto d = GetDevice();
  anari::release(d, m_handles->parameters.vertex.position);
  anari::release(d, m_handles->parameters.vertex.radius);
  m_handles->parameters.vertex.position = nullptr;
  m_handles->parameters.vertex.radius = nullptr;

  vtkm::Bounds coordBounds = coords.GetBounds();
  vtkm::Float64 lx = coordBounds.X.Length();
  vtkm::Float64 ly = coordBounds.Y.Length();
  vtkm::Float64 lz = coordBounds.Z.Length();
  vtkm::Float64 mag = vtkm::Sqrt(lx * lx + ly * ly + lz * lz);
  constexpr vtkm::Float64 heuristic = 300.;
  auto glyphSize = static_cast<vtkm::Float32>(mag / heuristic);

  auto arrays = makeGlyphs(field, cells, coords, glyphSize, m_offset);

  auto *v =
      (glm::vec3 *)arrays.vertices.GetBuffers()->ReadPointerHost(*arrays.token);
  auto *r = (float *)arrays.radii.GetBuffers()->ReadPointerHost(*arrays.token);

  m_handles->parameters.vertex.position = anari::newArray1D(
      d, v, noopANARIDeleter, nullptr, arrays.vertices.GetNumberOfValues());
  m_handles->parameters.vertex.radius = anari::newArray1D(
      d, r, noopANARIDeleter, nullptr, arrays.radii.GetNumberOfValues());
  m_handles->parameters.numPrimitives = numGlyphs;

  updateGeometry();

  m_arrays = arrays;
}

void ANARIMapperGlyphs::updateGeometry()
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
  anari::setParameter(d, m_handles->geometry, "caps", "both");
  anari::commit(d, m_handles->geometry);
}

ANARIMapperGlyphs::ANARIHandles::~ANARIHandles()
{
  anari::release(device, surface);
  anari::release(device, material);
  anari::release(device, geometry);
  anari::release(device, parameters.vertex.position);
  anari::release(device, parameters.vertex.radius);
  anari::release(device, device);
}

} // namespace vtkm_anari
