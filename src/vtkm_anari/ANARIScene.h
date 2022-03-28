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
  void AddMapper(const ANARIMapperType &mapper,
      const char *name = "<unnamed>",
      bool show = true);

  vtkm::IdComponent GetNumberOfMappers() const;

  const char *GetMapperName(vtkm::IdComponent id) const;
  void SetMapperName(vtkm::IdComponent id, const char *name);

  bool GetMapperShown(vtkm::IdComponent id) const;
  void SetMapperShown(vtkm::IdComponent id, bool shown);

  anari::Device GetDevice() const;

  anari::World GetANARIWorld();

 private:
  void updateWorld();

  anari::Device m_device{nullptr};
  anari::World m_world{nullptr};

  struct Mapper
  {
    std::unique_ptr<ANARIMapper> mapper;
    std::string name;
    bool show{true};
  };

  std::vector<Mapper> m_mappers;
};

// Inlined definitions ////////////////////////////////////////////////////////

template <typename ANARIMapperType>
inline void ANARIScene::AddMapper(
    const ANARIMapperType &mapper, const char *name, bool show)
{
  static_assert(std::is_base_of<ANARIMapper, ANARIMapperType>::value,
      "Only ANARIMapper types can be added to ANARIScene");
  m_mappers.push_back({std::make_unique<ANARIMapperType>(mapper), name, true});
  updateWorld();
}

} // namespace vtkm_anari