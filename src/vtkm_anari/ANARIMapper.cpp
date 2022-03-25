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

ANARIMapper::ANARIMapper(
    anari::Device device, const ANARIActor &actor, const ColorTable &colorTable)
    : m_device(device), m_actor(actor), m_colorTable(colorTable)
{
  anari::retain(m_device, m_device);
}

ANARIMapper::~ANARIMapper()
{
  anari::release(m_device, m_device);
}

anari::Device ANARIMapper::GetDevice() const
{
  return m_device;
}

const ANARIActor &ANARIMapper::GetActor() const
{
  return m_actor;
}

const ColorTable &ANARIMapper::GetColorTable() const
{
  return m_colorTable;
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
  if (!m_group) {
    m_group = anari::newObject<anari::Group>(m_device);

    auto surface = GetANARISurface();
    if (surface) {
      anari::setParameter(
          m_device, m_group, "surface", anari::newArray1D(m_device, &surface));
    }

    auto volume = GetANARIVolume();
    if (volume) {
      anari::setParameter(
          m_device, m_group, "volume", anari::newArray1D(m_device, &volume));
    }

    anari::commit(m_device, m_group);
  }

  return m_group;
}

anari::Instance ANARIMapper::GetANARIInstance()
{
  if (!m_instance) {
    m_instance = anari::newObject<anari::Instance>(m_device);
    auto group = GetANARIGroup();
    anari::setParameter(m_device, m_instance, "group", group);
    anari::commit(m_device, m_instance);
  }

  return m_instance;
}

} // namespace vtkm_anari
