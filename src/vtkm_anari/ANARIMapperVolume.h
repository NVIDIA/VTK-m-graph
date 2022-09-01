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

namespace vtkm_anari {

struct VolumeParameters
{
  anari::Array3D data{nullptr};
  int dims[3];
  float origin[3];
  float spacing[3];
};

struct VolumeArrays
{
  vtkm::cont::ArrayHandle<vtkm::Float32> data;
  std::shared_ptr<vtkm::cont::Token> token{new vtkm::cont::Token};
};

struct VTKM_ANARI_EXPORT ANARIMapperVolume : public ANARIMapper
{
  ANARIMapperVolume(anari::Device device,
      const ANARIActor &actor,
      const char *name = "<volume>",
      const ColorTable &colorTable = ColorTable::Preset::Default);

  void SetActor(const ANARIActor &actor) override;

  void SetANARIColorMapArrays(anari::Array1D color,
      anari::Array1D color_position,
      anari::Array1D opacity,
      anari::Array1D opacity_position,
      bool releaseArrays = true) override;

  void SetANARIColorMapValueRange(const vtkm::Vec2f_32 &valueRange) override;
  void SetANARIColorMapOpacityScale(vtkm::Float32 opacityScale) override;

  const VolumeParameters &Parameters();

  anari::SpatialField GetANARISpatialField() override;
  anari::Volume GetANARIVolume() override;

 private:
  void constructArrays(bool regenerate = false);
  void updateSpatialField();

  struct ANARIHandles
  {
    anari::Device device{nullptr};
    anari::SpatialField spatialField{nullptr};
    anari::Volume volume{nullptr};
    VolumeParameters parameters;
    ~ANARIHandles();
    void releaseArrays();
  };

  std::shared_ptr<ANARIHandles> m_handles;
  VolumeArrays m_arrays;
};

} // namespace vtkm_anari