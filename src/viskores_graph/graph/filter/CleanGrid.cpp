// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// viskores
#include <viskores/filter/clean_grid/CleanGrid.h>

namespace viskores {
namespace graph {

CleanGridNode::CleanGridNode()
{
  addParameter({this, "compactPointFields", ParameterType::BOOL, true});
  addParameter({this, "mergePoints", ParameterType::BOOL, true});
  addParameter({this, "fastMerge", ParameterType::BOOL, false});
  addParameter({this, "removeDegenerateCells", ParameterType::BOOL, true});
}

const char *CleanGridNode::kind() const
{
  return "CleanGrid";
}

void CleanGridNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "compactPointFields"))
      m_compactPointFields = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "mergePoints"))
      m_mergePoints = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "fastMerge"))
      m_fastMerge = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "removeDegenerateCells"))
      m_removeDegenerateCells = p->valueAs<bool>();
  }

  markChanged();
}

cont::DataSet CleanGridNode::execute()
{
  auto ds = getDataSetFromPort(datasetInput());

  filter::clean_grid::CleanGrid filter;
  filter.SetCompactPointFields(m_compactPointFields);
  filter.SetMergePoints(m_mergePoints);
  filter.SetFastMerge(m_fastMerge);
  filter.SetRemoveDegenerateCells(m_removeDegenerateCells);
  filter.SetToleranceIsAbsolute(true);
  filter.SetTolerance(std::nextafter(Epsilon<FloatDefault>(),
      2.f * Epsilon<FloatDefault>()));

  return filter.Execute(ds);
}

} // namespace graph
} // namespace viskores
