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
#include <vtkm/cont/CoordinateSystem.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/Field.h>
#include <vtkm/cont/UnknownCellSet.h>
// std
#include <array>

#include "ANARIExports.h"

namespace vtkm_anari {

using FieldSet = std::array<vtkm::cont::Field, 4>;

enum PrimaryField
{
  FIELD1 = 0,
  FIELD2,
  FIELD3,
  FIELD4
};

struct VTKM_ANARI_EXPORT ANARIActor
{
  ANARIActor() = default;
  ANARIActor(const vtkm::cont::UnknownCellSet &cells,
      const vtkm::cont::CoordinateSystem &coordinates,
      const vtkm::cont::Field &field1 = {},
      const vtkm::cont::Field &field2 = {},
      const vtkm::cont::Field &field3 = {},
      const vtkm::cont::Field &field4 = {});
  ANARIActor(const vtkm::cont::UnknownCellSet &cells,
      const vtkm::cont::CoordinateSystem &coordinates,
      const FieldSet &fieldset);
  ANARIActor(const vtkm::cont::DataSet &dataset);

  const vtkm::cont::UnknownCellSet &GetCellSet() const;
  const vtkm::cont::CoordinateSystem &GetCoordinateSystem() const;
  const vtkm::cont::Field &GetField(int idx = 0) const;

  void SetPrimaryField(PrimaryField idx);
  PrimaryField GetPrimaryField() const;

  vtkm::cont::DataSet MakeDataSet() const;

 private:
  struct ActorData
  {
    vtkm::cont::UnknownCellSet cells;
    vtkm::cont::CoordinateSystem coordinates;
    FieldSet fields;
    PrimaryField primaryField{PrimaryField::FIELD1};
  };

  std::shared_ptr<ActorData> m_data{std::make_shared<ActorData>()};
};

} // namespace vtkm_anari