// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/contour/Contour.h>

namespace vtkm {
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

  Range range;
  auto field = ds.GetField(inPort->cselector()->fieldName());
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
} // namespace vtkm
