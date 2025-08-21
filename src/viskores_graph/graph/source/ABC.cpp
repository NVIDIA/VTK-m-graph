// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../SourceNode.h"
// vtk-m
#include <viskores/cont/DataSetBuilderUniform.h>
#include <viskores/worklet/WorkletMapField.h>

namespace viskores {
namespace graph {

class GenerateABCField : public worklet::WorkletMapField
{
 public:
  Float32 A{0.f};
  Float32 B{0.f};
  Float32 C{0.f};
  Int64 Dim{64};

  VISKORES_CONT
  GenerateABCField(float _A, float _B, float _C, Int64 _Dim)
      : A(_A), B(_B), C(_C), Dim(_Dim)
  {}

  using ControlSignature = void(WholeArrayOut);
  using ExecutionSignature = void(InputIndex, _1);

  template <typename OutFieldPortalType>
  VISKORES_EXEC void operator()(const Id idx, OutFieldPortalType &f) const
  {
    const Float32 factor = 2.f * Pi<Float32>() / this->Dim;

    auto i = idx;
    const auto z_idx = i / (Dim * Dim);
    i -= (z_idx * Dim * Dim);
    const auto y_idx = (i / Dim);
    const auto x_idx = (i % Dim);

    const auto z = factor * z_idx;
    const auto y = factor * y_idx;
    const auto x = factor * x_idx;

    const auto a = this->A * Sin(z) + this->C * Cos(y);
    const auto b = this->B * Sin(x) + this->A * Cos(z);
    const auto c = this->C * Sin(y) + this->B * Cos(x);

    f.Set(idx, Vec3f_32(a, b, c));
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

  markChanged();
}

cont::DataSet ABCSourceNode::execute()
{
  auto size = parameter("size")->valueAs<int>();
  auto A = parameter("A")->valueAs<float>() * 2.f * Pi<Float32>();
  auto B = parameter("B")->valueAs<float>() * 2.f * Pi<Float32>();
  auto C = parameter("C")->valueAs<float>() * 2.f * Pi<Float32>();

  cont::DataSetBuilderUniform builder;
  auto dataset = builder.Create(Id3(size),
      Vec3f(-1.f),
      Vec3f(2.f / Float32(size)));

  cont::ArrayHandle<Vec3f_32> field;
  field.Allocate(dataset.GetCoordinateSystem().GetNumberOfValues());

  GenerateABCField worklet(A, B, C, size);
  worklet::DispatcherMapField<GenerateABCField> dispatch(worklet);
  dispatch.Invoke(field);

  dataset.AddField(cont::Field(
      "Velocity", cont::Field::Association::Points, field));

  return dataset;
}

} // namespace graph
} // namespace viskores