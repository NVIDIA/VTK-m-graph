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

#pragma once

#include "ANARIMapper.h"
// std
#include <string>
#include <type_traits>

namespace vtkm_anari {

struct VTKM_ANARI_EXPORT ANARIScene
{
  ANARIScene(anari::Device device);
  virtual ~ANARIScene();

  ANARIScene(const ANARIScene &) = delete;
  ANARIScene(ANARIScene &&) = delete;

  ANARIScene &operator=(const ANARIScene &) = delete;
  ANARIScene &operator=(ANARIScene &&) = delete;

  template <typename ANARIMapperType>
  ANARIMapperType &AddMapper(
      const ANARIMapperType &mapper, bool visible = true);

  template <typename ANARIMapperType>
  void ReplaceMapper(
      const ANARIMapperType &newMapper, vtkm::IdComponent id, bool visible);

  vtkm::IdComponent GetNumberOfMappers() const;
  bool HasMapperWithName(const char *name) const;
  vtkm::IdComponent GetMapperIndexByName(const char *name);

  ANARIMapper &GetMapper(vtkm::IdComponent id);
  ANARIMapper &GetMapper(const char *name);

  bool GetMapperVisible(vtkm::IdComponent id) const;
  void SetMapperVisible(vtkm::IdComponent id, bool shown);

  void RemoveMapper(vtkm::IdComponent id);
  void RemoveMapper(const char *name);

  void RemoveAllMappers();

  anari::Device GetDevice() const;

  anari::World GetANARIWorld();

 private:
  void updateWorld();

  anari::Device m_device{nullptr};
  anari::World m_world{nullptr};

  struct Mapper
  {
    std::unique_ptr<ANARIMapper> mapper;
    bool show{true};
  };

  std::vector<Mapper> m_mappers;
};

// Inlined definitions ////////////////////////////////////////////////////////

template <typename ANARIMapperType>
inline ANARIMapperType &ANARIScene::AddMapper(
    const ANARIMapperType &mapper, bool visible)
{
  static_assert(std::is_base_of<ANARIMapper, ANARIMapperType>::value,
      "Only ANARIMapper types can be added to ANARIScene");

  auto *name = mapper.GetName();
  if (HasMapperWithName(name)) {
    auto idx = GetMapperIndexByName(name);
    ReplaceMapper(mapper, idx, visible);
    return (ANARIMapperType &)GetMapper(idx);
  } else {
    m_mappers.push_back({std::make_unique<ANARIMapperType>(mapper), visible});
    updateWorld();
    return (ANARIMapperType &)GetMapper(GetNumberOfMappers() - 1);
  }
}

template <typename ANARIMapperType>
inline void ANARIScene::ReplaceMapper(
    const ANARIMapperType &newMapper, vtkm::IdComponent id, bool visible)
{
  static_assert(std::is_base_of<ANARIMapper, ANARIMapperType>::value,
      "Only ANARIMapper types can be added to ANARIScene");
  const bool wasVisible = GetMapperVisible(id);
#if 1
  m_mappers[id] = {std::make_unique<ANARIMapperType>(newMapper), visible};
#else
  m_mappers[id] = newMapper;
  SetMapperVisible(id, visible);
#endif
  if (wasVisible || visible)
    updateWorld();
}

} // namespace vtkm_anari