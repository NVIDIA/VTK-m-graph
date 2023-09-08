// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "FieldSelector.h"

namespace vtkm {
namespace graph {

void FieldSelector::setFieldNames(
    cont::DataSet ds, bool includeCoordinateSystems)
{
  m_fields.clear();
  for (size_t i = 0; i < ds.GetNumberOfFields(); i++) {
    auto name = ds.GetField(i).GetName();
    bool doInsert = includeCoordinateSystems || !ds.HasCoordinateSystem(name);
    if (doInsert)
      m_fields.push_back(name);
  }
  m_fields.push_back("[none]");
  if (m_currentField >= m_fields.size())
    m_currentField = 0;
}

size_t FieldSelector::numFields() const
{
  return m_fields.size();
}

const char *FieldSelector::fieldName() const
{
  return fieldName(currentField());
}

const char *FieldSelector::fieldName(size_t i) const
{
  return m_fields[i].c_str();
}

size_t FieldSelector::currentField() const
{
  return m_currentField;
}

void FieldSelector::setCurrentField(size_t i)
{
  m_currentField = i;
}

} // namespace graph
} // namespace vtkm
