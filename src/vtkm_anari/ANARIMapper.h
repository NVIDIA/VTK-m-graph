// Copyright 2022 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

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
  TRIANGLES
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
  SURFACE
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

struct RenderableObject
{
  RenderableObjectType type;
  union ObjectHandle
  {
    anari::Volume volume;
    anari::Surface surface;
  } object;
};

// Main mapper function ///////////////////////////////////////////////////////

VTKM_ANARI_INTERFACE RenderableObject makeANARIObject(
    anari::Device d, Actor actor);

} // namespace vtkm_anari