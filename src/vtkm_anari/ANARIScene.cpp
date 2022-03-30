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

#include "ANARIScene.h"
// std
#include <algorithm>

namespace vtkm_anari {

ANARIScene::ANARIScene(anari::Device device) : m_device(device)
{
  anari::retain(m_device, m_device);
}

ANARIScene::~ANARIScene()
{
  anari::release(m_device, m_world);
  anari::release(m_device, m_device);
}

vtkm::IdComponent ANARIScene::GetNumberOfMappers() const
{
  return static_cast<vtkm::IdComponent>(m_mappers.size());
}

ANARIMapper &ANARIScene::GetMapper(vtkm::IdComponent id)
{
  return *m_mappers[id].mapper;
}

bool ANARIScene::GetMapperVisible(vtkm::IdComponent id) const
{
  return m_mappers[id].show;
}

void ANARIScene::SetMapperVisible(vtkm::IdComponent id, bool shown)
{
  auto &m = m_mappers[id];
  if (m.show != shown) {
    m.show = shown;
    updateWorld();
  }
}

void ANARIScene::RemoveMapper(vtkm::IdComponent id)
{
  m_mappers.erase(m_mappers.begin() + id);
  updateWorld();
}

void ANARIScene::RemoveMapper(const char *name)
{
  std::string n = name;
  m_mappers.erase(std::remove_if(m_mappers.begin(),
                      m_mappers.end(),
                      [&](auto &m) { return m.mapper->GetName() == n; }),
      m_mappers.end());
  updateWorld();
}

void ANARIScene::RemoveAllMappers()
{
  m_mappers.clear();
  updateWorld();
}

anari::Device ANARIScene::GetDevice() const
{
  return m_device;
}

anari::World ANARIScene::GetANARIWorld()
{
  if (!m_world) {
    auto d = GetDevice();
    m_world = anari::newObject<anari::World>(d);
    updateWorld();
  }

  return m_world;
}

void ANARIScene::updateWorld()
{
  if (!m_world)
    return; // nobody has asked for the world yet, so don't actually do anything

  auto d = GetDevice();

  std::vector<anari::Instance> instances;

  for (auto &m : m_mappers) {
    auto i = m.mapper->GetANARIInstance();
    if (i && m.show)
      instances.push_back(i);
  }

  if (!instances.empty()) {
    anari::setAndReleaseParameter(d,
        m_world,
        "instance",
        anari::newArray1D(d, instances.data(), instances.size()));
  } else
    anari::unsetParameter(d, m_world, "instance");

  anari::commit(d, m_world);
}

} // namespace vtkm_anari
