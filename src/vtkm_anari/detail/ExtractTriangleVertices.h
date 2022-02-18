/*
 * Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <vtkm/worklet/WorkletMapField.h>

namespace vtkm_anari {

class ExtractTriangleVertices : public vtkm::worklet::WorkletMapField
{
 public:
  VTKM_CONT
  ExtractTriangleVertices() = default;

  typedef void ControlSignature(FieldIn, WholeArrayIn, WholeArrayOut);
  typedef void ExecutionSignature(InputIndex, _1, _2, _3);

  template <typename PointPortalType, typename OutPortalType>
  VTKM_EXEC void operator()(const vtkm::Id idx,
      const vtkm::Id4 indices,
      const PointPortalType &points,
      OutPortalType &out) const
  {
    out.Set(3 * idx + 0, static_cast<vtkm::Vec3f_32>(points.Get(indices[1])));
    out.Set(3 * idx + 1, static_cast<vtkm::Vec3f_32>(points.Get(indices[2])));
    out.Set(3 * idx + 2, static_cast<vtkm::Vec3f_32>(points.Get(indices[3])));
  }
};

} // namespace vtkm_anari