// Copyright 2022 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// anari
#include <anari/anari_cpp.hpp>
// vtk-m
#include <vtkm/cont/DataSet.h>
// std
#include <variant>

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

RenderableObject makeANARIObject(anari::Device d, Actor actor);

} // namespace vtkm_anari