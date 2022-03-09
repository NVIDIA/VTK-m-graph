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

// anari
#include <anari/anari_cpp.hpp>
// vtk-m
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/Field.h>

#ifdef _WIN32
#ifdef VTKM_ANARI_STATIC_DEFINE
#define VTKM_ANARI_INTERFACE
#else
#ifdef vtkm_anari_EXPORTS
#define VTKM_ANARI_INTERFACE __declspec(dllexport)
#else
#define VTKM_ANARI_INTERFACE __declspec(dllimport)
#endif
#endif
#elif defined __GNUC__
#define VTKM_ANARI_INTERFACE __attribute__((__visibility__("default")))
#else
#define VTKM_ANARI_INTERFACE
#endif

namespace vtkm_anari {

struct Actor
{
  vtkm::cont::DataSet dataset;
  vtkm::cont::Field field;
};

struct VTKM_ANARI_INTERFACE ANARIMapper
{
  ANARIMapper(anari::Device device, Actor actor);
  virtual ~ANARIMapper();

  ANARIMapper(const ANARIMapper &) = delete;
  ANARIMapper(ANARIMapper &&) = delete;

  ANARIMapper &operator=(const ANARIMapper &) = delete;
  ANARIMapper &operator=(ANARIMapper &&) = delete;

 protected:
  anari::Device m_device{nullptr};
  Actor m_actor;
};

} // namespace vtkm_anari