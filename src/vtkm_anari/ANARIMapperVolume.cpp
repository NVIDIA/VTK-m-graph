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

ANARIMapperVolume::ANARIMapperVolume(
    anari::Device device, const ANARIActor &actor)
    : ANARIMapper(device, actor)
{}

ANARIMapperVolume::~ANARIMapperVolume()
{
  anari::release(GetDevice(), m_parameters.data);
}

const VolumeParameters &ANARIMapperVolume::Parameters()
{
  constructParameters();
  return m_parameters;
}

anari::SpatialField ANARIMapperVolume::MakeField()
{
  constructParameters();
  if (!m_parameters.data)
    return nullptr;

  auto d = GetDevice();
  auto field = anari::newObject<anari::SpatialField>(d, "structuredRegular");
  anari::setParameter(d, field, "origin", m_parameters.origin);
  anari::setParameter(d, field, "spacing", m_parameters.spacing);
  anari::setParameter(d, field, "data", m_parameters.data);
  anari::commit(d, field);
  return field;
}

void ANARIMapperVolume::constructParameters()
{
  if (m_parameters.data)
    return;

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

    std::memcpy(m_parameters.dims, &dims, sizeof(dims));
    std::memcpy(m_parameters.origin, &bLower, sizeof(bLower));
    std::memcpy(m_parameters.spacing, &spacing, sizeof(spacing));
    m_parameters.data = anari::newArray3D(d, ptr, dims.x, dims.y, dims.z);
  }
}

} // namespace vtkm_anari
