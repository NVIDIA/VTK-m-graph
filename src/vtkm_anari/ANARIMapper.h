// Copyright 2022 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// anari
#include <anari/anari_cpp.hpp>
// vtk-m
#include <vtkm/cont/DataSet.h>
// std
#include <variant>

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

// Field id or name
using FieldIndex = std::variant<vtkm::Id, std::string>;

struct Actor
{
  vtkm::cont::DataSet dataset;
  Representation representation;
  FieldIndex field;
};

using RenderableObject =
    std::variant<std::monostate, anari::Volume, anari::Surface>;

// Main mapper function ///////////////////////////////////////////////////////

VTKM_ANARI_INTERFACE RenderableObject makeANARIObject(anari::Device d, Actor actor);

} // namespace vtkm_anari