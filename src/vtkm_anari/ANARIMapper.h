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

// Types //////////////////////////////////////////////////////////////////////

enum class Representation
{
  VOLUME,
  SURFACE
};

enum class FieldIndexType
{
  ID,
  STRING
};

enum class RenderableObjectType
{
  EMPTY,
  VOLUME,
  TRIANGLES
};

struct VTKM_ANARI_INTERFACE FieldIndex
{
  FieldIndex() = default;
  FieldIndex(vtkm::Id id);
  FieldIndex(std::string name);

  FieldIndexType type;
  // No union for the following members; nontrivial constructor/destructor
  vtkm::Id id;
  std::string name;
};

struct Actor
{
  vtkm::cont::DataSet dataset;
  Representation representation;
  FieldIndex fieldIndex;
};

namespace renderable {

struct Geometry
{
  unsigned int numPrimitives;
};

struct Triangles : public Geometry
{
  struct VertexData
  {
    anari::Array1D position{nullptr};
    anari::Array1D attribute{nullptr};
  } vertex{};

  struct PrimitiveData
  {
    anari::Array1D index{nullptr};
    anari::Array1D attribute{nullptr};
  } primitive{};
};

struct Volume
{
  anari::Array3D data{nullptr};
  size_t dims[3];
  float origin[3];
  float spacing[3];
};

struct Object
{
  anari::Device device{nullptr};
  RenderableObjectType type{RenderableObjectType::EMPTY};
  union ObjectArrays
  {
    renderable::Volume volume;
    renderable::Triangles triangles{};
  } object{};
};

} // namespace renderable

// Main mapper function ///////////////////////////////////////////////////////

VTKM_ANARI_INTERFACE renderable::Object makeANARIObject(
    anari::Device d, Actor actor);

// Cleanup helper /////////////////////////////////////////////////////////////

VTKM_ANARI_INTERFACE void releaseHandles(renderable::Object o);

} // namespace vtkm_anari