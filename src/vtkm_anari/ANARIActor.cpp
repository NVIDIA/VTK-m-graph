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

#include "ANARIActor.h"

namespace vtkm_anari {

ANARIActor::ANARIActor(const vtkm::cont::UnknownCellSet &cells,
    const vtkm::cont::CoordinateSystem &coordinates,
    const vtkm::cont::Field &field1,
    const vtkm::cont::Field &field2,
    const vtkm::cont::Field &field3,
    const vtkm::cont::Field &field4)
{
  m_data->cells = cells;
  m_data->coordinates = coordinates;
  m_data->fields[0] = field1;
  m_data->fields[1] = field2;
  m_data->fields[2] = field3;
  m_data->fields[3] = field4;
}

ANARIActor::ANARIActor(const vtkm::cont::UnknownCellSet &cells,
    const vtkm::cont::CoordinateSystem &coordinates,
    const FieldSet &f)
    : ANARIActor(cells, coordinates, f[0], f[1], f[2], f[3])
{}

ANARIActor::ANARIActor(const vtkm::cont::DataSet &dataset)
{
  m_data->cells = dataset.GetCellSet();
  m_data->coordinates = dataset.GetCoordinateSystem();
  for (int i = 0; i < dataset.GetNumberOfFields() && i < 4; i++)
    m_data->fields[i] = dataset.GetField(i);
}

const vtkm::cont::UnknownCellSet &ANARIActor::GetCellSet() const
{
  return m_data->cells;
}

const vtkm::cont::CoordinateSystem &ANARIActor::GetCoordinateSystem() const
{
  return m_data->coordinates;
}

const vtkm::cont::Field &ANARIActor::GetField(int idx) const
{
  return m_data->fields[idx];
}

void ANARIActor::SetPrimaryField(PrimaryField idx)
{
  m_data->primaryField = idx;
}

PrimaryField ANARIActor::GetPrimaryField() const
{
  return m_data->primaryField;
}

vtkm::cont::DataSet ANARIActor::MakeDataSet() const
{
  vtkm::cont::DataSet dataset;
  dataset.SetCellSet(GetCellSet());
  dataset.AddCoordinateSystem(GetCoordinateSystem());
  dataset.AddField(GetField());
  return dataset;
}

} // namespace vtkm_anari
