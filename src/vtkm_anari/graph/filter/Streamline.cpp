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

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/Streamline.h>
#include <vtkm/worklet/WorkletMapField.h>
// std
#include <cmath>
#include <cstring>

namespace vtkm_anari {
namespace graph {

// Worklets ///////////////////////////////////////////////////////////////////

class CoordinateSystemToParticles : public vtkm::worklet::WorkletMapField
{
 public:
  VTKM_CONT CoordinateSystemToParticles() = default;

  using ControlSignature = void(FieldIn, FieldOut);
  using ExecutionSignature = void(InputIndex, _1, _2);

  VTKM_EXEC void operator()(const vtkm::Id out_idx,
      const vtkm::Vec3f_32 &in_coord,
      vtkm::Particle &outP) const
  {
    outP = vtkm::Particle(in_coord, out_idx);
  }
};

// StreamlineNode definitions /////////////////////////////////////////////////

StreamlineNode::StreamlineNode()
{
  addParameter({this, "steps", ParameterType::INT, 100});
  addParameter({this, "stepSize", ParameterType::FLOAT, 0.f});
}

const char *StreamlineNode::kind() const
{
  return "Streamline";
}

size_t StreamlineNode::numInput() const
{
  return 2;
}

InPort *StreamlineNode::streamlineCoordsInput()
{
  return &m_coordsInPort;
}

void StreamlineNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "steps"))
      m_steps = p->valueAs<int>();
    if (!std::strcmp(p->name(), "stepSize"))
      m_stepSize = p->valueAs<float>();
  }

  markChanged();
}

vtkm::cont::DataSet StreamlineNode::execute()
{
  if (!datasetInput()->isConnected() || !streamlineCoordsInput()->isConnected())
    return {};

  auto ds = getDataSetFromPort(datasetInput());
  auto cds = getDataSetFromPort(streamlineCoordsInput());

  float stepSize = m_stepSize;
  auto coords = cds.GetCoordinateSystem();

  if (stepSize == 0.f) {
    const vtkm::Bounds bounds = coords.GetBounds();
    const float diagonal =
        std::sqrt(std::pow((float)(bounds.X.Max - bounds.X.Min), 2.f)
            + std::pow((float)(bounds.Y.Max - bounds.Y.Min), 2.f)
            + std::pow((float)(bounds.Z.Max - bounds.Z.Min), 2.f));
    stepSize = diagonal / 1000.f;
  }

  vtkm::cont::ArrayHandle<vtkm::Particle> seedArray;
  seedArray.Allocate(coords.GetNumberOfValues());
  CoordinateSystemToParticles worklet;
  vtkm::worklet::DispatcherMapField<CoordinateSystemToParticles>(worklet)
      .Invoke(coords, seedArray);

  vtkm::filter::Streamline filter;
  filter.SetActiveField(ds.GetField(0).GetName());
  filter.SetStepSize(stepSize);
  filter.SetNumberOfSteps(m_steps);
  filter.SetSeeds(seedArray);

  return filter.Execute(ds);
}

} // namespace graph
} // namespace vtkm_anari
