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

#include "../SourceNode.h"
// vtk-m
#include <vtkm/cont/DataSetBuilderUniform.h>
#include <vtkm/worklet/WorkletMapField.h>
// std
#include <cmath>

namespace vtkm_anari {
namespace graph {

class GenerateABCField : public vtkm::worklet::WorkletMapField
{
 public:
  vtkm::Float32 A{0.f};
  vtkm::Float32 B{0.f};
  vtkm::Float32 C{0.f};
  vtkm::Int64 Dim{64};

  VTKM_CONT
  GenerateABCField(float _A, float _B, float _C, vtkm::Int64 _Dim)
      : A(_A), B(_B), C(_C), Dim(_Dim)
  {}

  using ControlSignature = void(WholeArrayOut);
  using ExecutionSignature = void(InputIndex, _1);

  template <typename OutFieldPortalType>
  VTKM_EXEC void operator()(const vtkm::Id idx, OutFieldPortalType &f) const
  {
    const vtkm::Float32 factor = 2.f * M_PI / vtkm::Float32(this->Dim);

    auto i = idx;
    const auto z_idx = i / (Dim * Dim);
    i -= (z_idx * Dim * Dim);
    const auto y_idx = (i / Dim);
    const auto x_idx = (i % Dim);

    const auto z = factor * z_idx;
    const auto y = factor * y_idx;
    const auto x = factor * x_idx;

    const auto a = this->A * vtkm::Sin(z) + this->C * vtkm::Cos(y);
    const auto b = this->B * vtkm::Sin(x) + this->A * vtkm::Cos(z);
    const auto c = this->C * vtkm::Sin(y) + this->B * vtkm::Cos(x);

    f.Set(idx, vtkm::Vec3f_32(a, b, c));
  }
};

// ABCSourceNode definitions //////////////////////////////////////////////////

ABCSourceNode::ABCSourceNode()
{
  addParameter({this, "size", ParameterType::BOUNDED_INT, 64})
      ->setMinMax(8, 256, 64);
  addParameter({this, "A", ParameterType::BOUNDED_FLOAT, 0.5f})
      ->setMinMax(0.f, 1.f, 0.5f);
  addParameter({this, "B", ParameterType::BOUNDED_FLOAT, 0.5f})
      ->setMinMax(0.f, 1.f, 0.5f);
  addParameter({this, "C", ParameterType::BOUNDED_FLOAT, 0.5f})
      ->setMinMax(0.f, 1.f, 0.5f);
}

const char *ABCSourceNode::kind() const
{
  return "ABCSource";
}

void ABCSourceNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_MINMAX)
    return;

  m_needToGenerate = true;
  notifyObserver();
}

vtkm::cont::DataSet ABCSourceNode::dataset()
{
  if (m_needToGenerate) {
    auto size = parameter("size")->valueAs<int>();
    auto A = parameter("A")->valueAs<float>() * 2.f * M_PI;
    auto B = parameter("B")->valueAs<float>() * 2.f * M_PI;
    auto C = parameter("C")->valueAs<float>() * 2.f * M_PI;

    vtkm::cont::DataSetBuilderUniform builder;
    m_dataset = builder.Create(vtkm::Id3(size),
        vtkm::Vec3f(-1.f),
        vtkm::Vec3f(2.f / vtkm::Float32(size)));

    vtkm::cont::ArrayHandle<vtkm::Vec3f_32> field;
    field.Allocate(m_dataset.GetCoordinateSystem().GetNumberOfValues());

    GenerateABCField worklet(A, B, C, size);
    vtkm::worklet::DispatcherMapField<GenerateABCField> dispatch(worklet);
    dispatch.Invoke(field);

    m_dataset.AddField(vtkm::cont::Field(
        "Velocity", vtkm::cont::Field::Association::POINTS, field));
  }
  return m_dataset;
}

} // namespace graph
} // namespace vtkm_anari