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

#include "ANARIActor.h"
// anari
#include <anari/anari_cpp.hpp>
// vtk-m
#include <vtkm/cont/ColorTable.h>

#include "vtkm_anari_export.h"

namespace vtkm_anari {

using ColorTable = vtkm::cont::ColorTable;

inline void noopANARIDeleter(void *, void *) {}

struct VTKM_ANARI_EXPORT ANARIMapper
{
  ANARIMapper(anari::Device device,
      const ANARIActor &actor,
      const char *name = "<noname>",
      const ColorTable &colorTable = ColorTable::Preset::Default);
  virtual ~ANARIMapper() = default;

  anari::Device GetDevice() const;
  const ANARIActor &GetActor() const;
  const char *GetName() const;
  const ColorTable &GetColorTable() const;

  void SetName(const char *name);
  void SetColorTable(const ColorTable &colorTable);
  virtual void SetActor(const ANARIActor &actor);

  virtual void SetMapFieldAsAttribute(bool enabled);
  bool GetMapFieldAsAttribute() const;

  virtual void SetANARIColorMapArrays(anari::Array1D color,
      anari::Array1D color_position,
      anari::Array1D opacity,
      anari::Array1D opacity_position,
      bool releaseArrays = true);

  virtual void SetANARIColorMapValueRange(const vtkm::Vec2f_32 &valueRange);
  virtual void SetANARIColorMapOpacityScale(vtkm::Float32 opacityScale);

  virtual anari::Geometry GetANARIGeometry();
  virtual anari::SpatialField GetANARISpatialField();

  virtual anari::Surface GetANARISurface();
  virtual anari::Volume GetANARIVolume();

  anari::Group GetANARIGroup();
  anari::Instance GetANARIInstance();

 protected:
  std::string makeObjectName(const char *suffix) const;

 private:
  struct ANARIHandles
  {
    anari::Device device{nullptr};
    anari::Group group{nullptr};
    anari::Instance instance{nullptr};
    ~ANARIHandles();
  };

  std::shared_ptr<ANARIHandles> m_handles;
  ANARIActor m_actor;
  vtkm::cont::ColorTable m_colorTable;
  std::string m_name;
  bool m_mapFieldAsAttribute{true};
};

} // namespace vtkm_anari