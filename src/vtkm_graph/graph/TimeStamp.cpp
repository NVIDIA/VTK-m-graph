// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "TimeStamp.h"

namespace vtkm {
namespace graph {

static size_t g_nextValue{1};

TimeStamp::TimeStamp()
{
  renew();
}

TimeStamp::TimeStamp(TimeStamp &&rhs)
{
  m_value = rhs.m_value;
  rhs.m_value = 0;
}

TimeStamp &TimeStamp::operator=(TimeStamp &&rhs)
{
  m_value = rhs.m_value;
  rhs.m_value = 0;
  return *this;
}

TimeStamp::operator size_t() const
{
  return m_value;
}

void TimeStamp::renew()
{
  m_value = g_nextValue++;
}

} // namespace graph
} // namespace vtkm
