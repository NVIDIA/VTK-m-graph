// Copyright 2022 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

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