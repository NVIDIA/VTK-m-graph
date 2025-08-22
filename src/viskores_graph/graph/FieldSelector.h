// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "viskores_graph_export.h"
// viskores
#include <viskores/cont/DataSet.h>
// std
#include <string>
#include <vector>

namespace viskores {
namespace graph {

struct VISKORES_GRAPH_EXPORT FieldSelector
{
  FieldSelector() = default;
  ~FieldSelector() = default;

  void setFieldNames(cont::DataSet ds, bool includeCoordinateSystems = false);

  size_t numFields() const;
  const char *fieldName() const; // name of currently set field
  const char *fieldName(size_t i) const;

  size_t currentField() const;
  void setCurrentField(size_t i);

 private:
  std::vector<std::string> m_fields;
  size_t m_currentField{0};
};

} // namespace graph
} // namespace viskores
