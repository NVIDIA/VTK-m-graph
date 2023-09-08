// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>

namespace vtkm {
namespace graph {

struct TimeStamp
{
  TimeStamp();

  TimeStamp(const TimeStamp &) = delete;
  TimeStamp &operator=(const TimeStamp &) = delete;

  TimeStamp(TimeStamp &&);
  TimeStamp &operator=(TimeStamp &&);

  operator size_t() const;

  void renew();

 private:
  size_t m_value{0};
};

// Inlined definitions ////////////////////////////////////////////////////////

inline static bool operator<(const TimeStamp &t1, const TimeStamp &t2)
{
  return static_cast<size_t>(t1) < static_cast<size_t>(t2);
}

inline static bool operator==(const TimeStamp &t1, const TimeStamp &t2)
{
  return static_cast<size_t>(t1) == static_cast<size_t>(t2);
}

} // namespace graph
} // namespace vtkm
