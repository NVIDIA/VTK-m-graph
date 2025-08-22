// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// viskores
#include <viskores/filter/contour/Contour.h>

namespace viskores {
namespace graph {

ContourNode::ContourNode()
{
  addParameter({this, "isovalue", ParameterType::BOUNDED_FLOAT, 0.f});
  addParameter({this, "computeNormals", ParameterType::BOOL, false});
}

const char *ContourNode::kind() const
{
  return "Contour";
}

void ContourNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  markChanged();
}

cont::DataSet ContourNode::execute()
{
  auto *inPort = datasetInput();
  auto ds = getDataSetFromPort(inPort);

  auto field = ds.GetField(inPort->cselector()->fieldName());
  if (field.GetData().GetNumberOfComponentsFlat() != 1)
    return {};

  Range range;
  field.GetRange(&range);

  auto *p = parameter("isovalue");
  p->setMinMax<float>(range.Min, range.Max, range.Center());

  filter::contour::Contour filter;
  filter.SetIsoValue(p->valueAs<float>());
  filter.SetActiveField(field.GetName());
  filter.SetGenerateNormals(parameter("computeNormals")->valueAs<bool>());

  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
