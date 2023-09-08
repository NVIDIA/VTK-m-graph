// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <vtkm/Range.h>

namespace vtkm {

template <typename T>
inline Float32 NormalizedFieldValue(const T &v, const Range &r)
{
  if (r.Length() >= 1e-4f) {
    return Clamp(
        (static_cast<Float32>(v) - r.Min) / r.Length(), 0, 1);
  } else
    return 0.f;
}

} // namespace vtkm
