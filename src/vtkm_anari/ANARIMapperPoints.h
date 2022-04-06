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

struct PointsParameters
{
  struct VertexData
  {
    anari::Array1D position{nullptr};
    anari::Array1D radius{nullptr};
    anari::Array1D attribute{nullptr};
  } vertex{};

  unsigned int numPrimitives{0};
};

struct PointsArrays
{
  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> vertices;
  vtkm::cont::ArrayHandle<vtkm::Float32> radii;
  vtkm::cont::ArrayHandle<vtkm::Float32> attribute;
  std::shared_ptr<vtkm::cont::Token> token{new vtkm::cont::Token};
};

struct VTKM_ANARI_EXPORT ANARIMapperPoints : public ANARIMapper
{
  ANARIMapperPoints(anari::Device device,
      const ANARIActor &actor,
      const char *name = "<points>",
      const ColorTable &colorTable = ColorTable::Preset::Default);

  void SetActor(const ANARIActor &actor) override;

  void SetANARIColorMapArrays(anari::Array1D color,
      anari::Array1D color_position,
      anari::Array1D opacity,
      anari::Array1D opacity_position,
      bool releaseArrays = true) override;

  void SetANARIColorMapValueRange(const vtkm::Vec2f_32 &valueRange) override;

  const PointsParameters &Parameters();

  anari::Geometry GetANARIGeometry() override;
  anari::Surface GetANARISurface() override;

 private:
  void constructParameters(bool regenerate = false);
  void updateGeometry();

  struct ANARIHandles
  {
    anari::Device device{nullptr};
    anari::Geometry geometry{nullptr};
    anari::Sampler sampler{nullptr};
    anari::Material material{nullptr};
    anari::Surface surface{nullptr};
    PointsParameters parameters;
    ~ANARIHandles();
  };

  std::shared_ptr<ANARIHandles> m_handles;
  PointsArrays m_arrays;
};

} // namespace vtkm_anari