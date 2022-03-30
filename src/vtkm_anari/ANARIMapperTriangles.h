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

struct TrianglesParameters
{
  struct VertexData
  {
    anari::Array1D position{nullptr};
    anari::Array1D normal{nullptr};
    anari::Array1D attribute{nullptr};
  } vertex{};

  struct PrimitiveData
  {
    anari::Array1D index{nullptr};
  } primitive{};

  unsigned int numPrimitives{0};
};

struct TriangleArrays
{
  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> vertices;
  vtkm::cont::ArrayHandle<vtkm::Float32> attribute;
  vtkm::cont::ArrayHandle<vtkm::Vec3f_32> normals;
};

struct VTKM_ANARI_EXPORT ANARIMapperTriangles : public ANARIMapper
{
  ANARIMapperTriangles(anari::Device device,
      const ANARIActor &actor,
      const char *name = "<triangles>",
      const ColorTable &colorTable = ColorTable::Preset::Default);

  void SetANARIColorMapArrays(anari::Array1D color,
      anari::Array1D color_position,
      anari::Array1D opacity,
      anari::Array1D opacity_position,
      bool releaseArrays = true) override;

  void SetANARIColorMapValueRange(const vtkm::Vec2f_32 &valueRange) override;

  const TrianglesParameters &Parameters();

  void SetCalculateNormals(bool enabled);

  anari::Geometry GetANARIGeometry() override;
  anari::Surface GetANARISurface() override;

 private:
  bool needToGenerateData() const;
  void constructParameters();

  struct ANARIHandles
  {
    anari::Device device{nullptr};
    anari::Geometry geometry{nullptr};
    anari::Sampler sampler{nullptr};
    anari::Material material{nullptr};
    anari::Surface surface{nullptr};
    TrianglesParameters parameters;
    ~ANARIHandles();
  };

  std::shared_ptr<ANARIHandles> m_handles;

  bool m_calculateNormals{false};
  TriangleArrays m_arrays;
};

} // namespace vtkm_anari