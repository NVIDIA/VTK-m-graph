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

#include "ANARIMapperVolume.h"
// anari + glm
#include <anari/anari_cpp/ext/glm.h>

namespace vtkm_anari {

ANARIMapperVolume::ANARIMapperVolume(anari::Device device,
    const ANARIActor &actor,
    const char *name,
    const ColorTable &colorTable)
    : ANARIMapper(device, actor, name, colorTable)
{
  m_handles = std::make_shared<ANARIMapperVolume::ANARIHandles>();
  m_handles->device = device;
  anari::retain(device, device);
}

void ANARIMapperVolume::SetActor(const ANARIActor &actor)
{
  ANARIMapper::SetActor(actor);
  constructParameters(true);
}

void ANARIMapperVolume::SetANARIColorMapArrays(anari::Array1D color,
    anari::Array1D color_position,
    anari::Array1D opacity,
    anari::Array1D opacity_position,
    bool releaseArrays)
{
  auto d = GetDevice();
  auto v = GetANARIVolume();
  anari::setParameter(d, v, "color", color);
  anari::setParameter(d, v, "color.position", color_position);
  anari::setParameter(d, v, "opacity", opacity);
  anari::setParameter(d, v, "opacity.position", opacity_position);
  anari::commit(d, v);
  ANARIMapper::SetANARIColorMapArrays(
      color, color_position, opacity, opacity_position, releaseArrays);
}

void ANARIMapperVolume::SetANARIColorMapValueRange(
    const vtkm::Vec2f_32 &valueRange)
{
  auto d = GetDevice();
  auto v = GetANARIVolume();
  anari::setParameter(d, v, "valueRange", ANARI_FLOAT32_BOX1, &valueRange);
  anari::commit(d, v);
}

void ANARIMapperVolume::SetANARIColorMapOpacityScale(vtkm::Float32 opacityScale)
{
  auto d = GetDevice();
  auto v = GetANARIVolume();
  anari::setParameter(d, v, "densityScale", opacityScale);
  anari::commit(d, v);
}

const VolumeParameters &ANARIMapperVolume::Parameters()
{
  constructParameters();
  return m_handles->parameters;
}

anari::SpatialField ANARIMapperVolume::GetANARISpatialField()
{
  constructParameters();
  if (!m_handles->parameters.data)
    return nullptr;

  auto d = GetDevice();
  m_handles->spatialField =
      anari::newObject<anari::SpatialField>(d, "structuredRegular");
  anari::setParameter(
      d, m_handles->spatialField, "name", makeObjectName("spatialField"));
  updateSpatialField();
  return m_handles->spatialField;
}

anari::Volume ANARIMapperVolume::GetANARIVolume()
{
  if (!m_valid)
    return nullptr;

  if (m_handles->volume)
    return m_handles->volume;

  auto spatialField = GetANARISpatialField();
  if (!spatialField)
    return nullptr;

  auto d = GetDevice();

  m_handles->volume = anari::newObject<anari::Volume>(d, "scivis");

  auto colorArray = anari::newArray1D(d, ANARI_FLOAT32_VEC3, 3);
  auto *colors = (glm::vec3 *)anari::map(d, colorArray);
  colors[0] = glm::vec3(1.f, 0.f, 0.f);
  colors[1] = glm::vec3(0.f, 1.f, 0.f);
  colors[2] = glm::vec3(0.f, 0.f, 1.f);
  anari::unmap(d, colorArray);

  auto opacityArray = anari::newArray1D(d, ANARI_FLOAT32, 2);
  auto *opacities = (float *)anari::map(d, opacityArray);
  opacities[0] = 0.f;
  opacities[1] = 1.f;
  anari::unmap(d, opacityArray);

  anari::setAndReleaseParameter(d, m_handles->volume, "color", colorArray);
  anari::setAndReleaseParameter(d, m_handles->volume, "opacity", opacityArray);
  anari::setParameter(d, m_handles->volume, "valueRange", glm::vec2(0.f, 10.f));
  anari::setParameter(d, m_handles->volume, "field", spatialField);
  anari::setParameter(d, m_handles->volume, "densityScale", 0.05f);
  anari::setParameter(d, m_handles->volume, "name", makeObjectName("volume"));
  anari::commit(d, m_handles->volume);

  return m_handles->volume;
}

void ANARIMapperVolume::constructParameters(bool regenerate)
{
  if (!regenerate && m_handles->parameters.data)
    return;

  m_valid = false;

  const auto &actor = GetActor();
  const auto &coords = actor.GetCoordinateSystem();
  const auto &cells = actor.GetCellSet();
  const auto &fieldArray = actor.GetField().GetData();

  auto d = GetDevice();

  if (!cells.IsSameType(vtkm::cont::CellSetStructured<3>()))
    printf("ANARIMapperVolume: CELLS ARE NOT STRUCTURED\n");
  else if (!fieldArray.IsType<vtkm::cont::ArrayHandle<vtkm::Float32>>())
    printf("ANARIMapperVolume: FIELD DATA NOT FLOAT32\n");
  else {
    auto structuredCells = cells.Cast<vtkm::cont::CellSetStructured<3>>();
    auto pdims = structuredCells.GetPointDimensions();

    VolumeArrays arrays;

    anari::release(d, m_handles->parameters.data);
    m_handles->parameters.data = nullptr;

    arrays.data =
        fieldArray.AsArrayHandle<vtkm::cont::ArrayHandle<vtkm::Float32>>();

    auto *ptr =
        (float *)arrays.data.GetBuffers()->ReadPointerHost(*arrays.token);

    auto bounds = coords.GetBounds();
    glm::vec3 bLower(bounds.X.Min, bounds.Y.Min, bounds.Z.Min);
    glm::vec3 bUpper(bounds.X.Max, bounds.Y.Max, bounds.Z.Max);
    glm::vec3 size = bUpper - bLower;

    glm::uvec3 dims(pdims[0], pdims[1], pdims[2]);
    auto spacing = size / (glm::vec3(dims) - 1.f);

    std::memcpy(m_handles->parameters.dims, &dims, sizeof(dims));
    std::memcpy(m_handles->parameters.origin, &bLower, sizeof(bLower));
    std::memcpy(m_handles->parameters.spacing, &spacing, sizeof(spacing));
    m_handles->parameters.data = anari::newArray3D(
        d, ptr, noopANARIDeleter, nullptr, dims.x, dims.y, dims.z);

    m_arrays = arrays;
    m_valid = true;
    updateSpatialField();
    refreshGroup();
  }
}

void ANARIMapperVolume::updateSpatialField()
{
  if (!m_handles->spatialField)
    return;
  auto d = GetDevice();
  anari::setParameter(
      d, m_handles->spatialField, "origin", m_handles->parameters.origin);
  anari::setParameter(
      d, m_handles->spatialField, "spacing", m_handles->parameters.spacing);
  anari::setParameter(
      d, m_handles->spatialField, "data", m_handles->parameters.data);
  anari::commit(d, m_handles->spatialField);
}

ANARIMapperVolume::ANARIHandles::~ANARIHandles()
{
  anari::release(device, volume);
  anari::release(device, spatialField);
  anari::release(device, parameters.data);
  anari::release(device, device);
}

} // namespace vtkm_anari
