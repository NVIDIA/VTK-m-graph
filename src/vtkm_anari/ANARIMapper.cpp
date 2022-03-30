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

namespace vtkm_anari {

ANARIMapper::ANARIMapper(anari::Device device,
    const ANARIActor &actor,
    const char *name,
    const ColorTable &colorTable)
    : m_actor(actor), m_colorTable(colorTable), m_name(name)
{
  m_dataToken = std::make_shared<vtkm::cont::Token>();
  m_handles = std::make_shared<ANARIHandles>();
  m_handles->device = device;
  anari::retain(device, device);
}

anari::Device ANARIMapper::GetDevice() const
{
  return m_handles->device;
}

const ANARIActor &ANARIMapper::GetActor() const
{
  return m_actor;
}

const char *ANARIMapper::GetName() const
{
  return m_name.c_str();
}

const ColorTable &ANARIMapper::GetColorTable() const
{
  return m_colorTable;
}

void ANARIMapper::SetANARIColorMapArrays(anari::Array1D color,
    anari::Array1D color_position,
    anari::Array1D opacity,
    anari::Array1D opacity_position,
    bool releaseArrays)
{
  auto d = GetDevice();
  if (releaseArrays) {
    anari::release(d, color);
    anari::release(d, color_position);
    anari::release(d, opacity);
    anari::release(d, opacity_position);
  }
}

void ANARIMapper::SetANARIColorMapValueRange(const vtkm::Vec2f_32 &)
{
  // no-op
}

void ANARIMapper::SetANARIColorMapOpacityScale(vtkm::Float32)
{
  // no-op
}

void ANARIMapper::SetName(const char *name)
{
  m_name = name;
}

void ANARIMapper::SetColorTable(const ColorTable &colorTable)
{
  m_colorTable = colorTable;
}

anari::Geometry ANARIMapper::GetANARIGeometry()
{
  return nullptr;
}

anari::SpatialField ANARIMapper::GetANARISpatialField()
{
  return nullptr;
}

anari::Surface ANARIMapper::GetANARISurface()
{
  return nullptr;
}

anari::Volume ANARIMapper::GetANARIVolume()
{
  return nullptr;
}

anari::Group ANARIMapper::GetANARIGroup()
{
  if (!m_handles->group) {
    auto d = GetDevice();

    m_handles->group = anari::newObject<anari::Group>(d);

    auto surface = GetANARISurface();
    if (surface) {
      anari::setAndReleaseParameter(
          d, m_handles->group, "surface", anari::newArray1D(d, &surface));
    }

    auto volume = GetANARIVolume();
    if (volume) {
      anari::setAndReleaseParameter(
          d, m_handles->group, "volume", anari::newArray1D(d, &volume));
    }

    anari::commit(d, m_handles->group);
  }

  return m_handles->group;
}

anari::Instance ANARIMapper::GetANARIInstance()
{
  if (!m_handles->instance) {
    auto d = GetDevice();
    m_handles->instance = anari::newObject<anari::Instance>(d);
    auto group = GetANARIGroup();
    anari::setParameter(d, m_handles->instance, "group", group);
    anari::commit(d, m_handles->instance);
  }

  return m_handles->instance;
}

vtkm::cont::Token &ANARIMapper::dataToken()
{
  return *m_dataToken;
}

ANARIMapper::ANARIHandles::~ANARIHandles()
{
  anari::release(device, group);
  anari::release(device, instance);
  anari::release(device, device);
}

} // namespace vtkm_anari
