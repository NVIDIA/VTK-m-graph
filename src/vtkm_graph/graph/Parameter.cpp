// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "Parameter.h"

namespace vtkm {
namespace graph {

const char *Parameter::name() const
{
  return m_name.c_str();
}

ParameterType Parameter::type() const
{
  return m_type;
}

bool Parameter::setRawValue(ParameterRawValue &&v)
{
  bool changed = false;

  if (!v.str.empty() && m_stringValue != v.str) {
    m_stringValue = std::move(v.str);
    changed = true;
  } else if (!std::equal(m_value.begin(), m_value.end(), v.buf.begin())) {
    std::copy(v.buf.begin(), v.buf.end(), m_value.begin());
    changed = true;
  }

  if (changed)
    notifyObserver(ParameterChangeType::NEW_VALUE);

  return changed;
}

void Parameter::unsetMinMax()
{
  std::fill(m_min.begin(), m_min.end(), 0);
  std::fill(m_max.begin(), m_max.end(), 0);
  m_hasMinMax = false;
}

bool Parameter::hasMinMax() const
{
  return m_hasMinMax;
}

void Parameter::notifyObserver(ParameterChangeType type)
{
  if (m_observer)
    m_observer->parameterChanged(this, type);
}

} // namespace graph
} // namespace vtkm