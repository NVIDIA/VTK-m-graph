// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>

#include "viskores_graph_export.h"

namespace viskores {
namespace graph {

struct VISKORES_GRAPH_EXPORT TimeStamp
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
} // namespace viskores
