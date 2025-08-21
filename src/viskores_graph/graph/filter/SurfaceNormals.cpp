// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <viskores/filter/vector_analysis/SurfaceNormals.h>
// std
#include <cstring>

namespace viskores {
namespace graph {

SurfaceNormalsNode::SurfaceNormalsNode()
{
  addParameter({this, "oriented", ParameterType::BOOL, true});
  addParameter({this, "flip", ParameterType::BOOL, true});
}

const char *SurfaceNormalsNode::kind() const
{
  return "SurfaceNormals";
}

void SurfaceNormalsNode::parameterChanged(
    Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "oriented"))
      m_oriented = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "flip"))
      m_flip = p->valueAs<bool>();
  }

  markChanged();
}

cont::DataSet SurfaceNormalsNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::vector_analysis::SurfaceNormals filter;
  filter.SetAutoOrientNormals(m_oriented);
  filter.SetFlipNormals(m_flip);
  filter.SetOutputFieldName("Normals");

  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
