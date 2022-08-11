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
#include <vtkm/worklet/WorkletMapTopology.h>
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

class CalculateIntegrationTime
    : public vtkm::worklet::WorkletVisitCellsWithPoints
{
 public:
  float StepSize{1.f};

  VTKM_CONT
  CalculateIntegrationTime(float stepSize) : StepSize(stepSize) {}

  using ControlSignature = void(CellSetIn, WholeArrayOut);
  using ExecutionSignature = void(CellShape, PointCount, PointIndices, _2);
  using InputDomain = _1;

  template <typename CellShapeTag,
      typename InIndicesType,
      typename OutTimePortalType>
  VTKM_EXEC void operator()(const CellShapeTag &shapeType,
      const vtkm::IdComponent &numPoints,
      const InIndicesType &indices,
      OutTimePortalType &outTime) const
  {
    for (int i = 0; i < numPoints; ++i) {
      outTime.Set(indices[i],
          shapeType.Id == vtkm::CELL_SHAPE_POLY_LINE ? i * StepSize : 0.f);
    }
  }
};

// Helper functions ///////////////////////////////////////////////////////////

static void annotateIntegrationTime(vtkm::cont::DataSet &ds, float stepSize)
{
  if (!ds.GetCellSet().IsType<vtkm::cont::CellSetExplicit<>>())
    return;

  auto cells = ds.GetCellSet().AsCellSet<vtkm::cont::CellSetExplicit<>>();

  vtkm::cont::ArrayHandle<float> integrationTimeFieldArray;
  integrationTimeFieldArray.Allocate(ds.GetNumberOfPoints());

  CalculateIntegrationTime worklet(stepSize);
  vtkm::worklet::DispatcherMapTopology<CalculateIntegrationTime>(worklet)
      .Invoke(cells, integrationTimeFieldArray);

  ds.AddField(vtkm::cont::make_FieldPoint(
      "integrationTime", integrationTimeFieldArray));
}

// StreamlineNode definitions /////////////////////////////////////////////////

StreamlineNode::StreamlineNode()
{
  addParameter({this, "steps", ParameterType::INT, 100});
  addParameter({this, "stepSize", ParameterType::FLOAT, 0.f});
  addParameter({this, "calculateTime", ParameterType::BOOL, true});
}

const char *StreamlineNode::kind() const
{
  return "Streamline";
}

InPort *StreamlineNode::inputBegin()
{
  return &m_mainDataInPort;
}

size_t StreamlineNode::numInput() const
{
  return 2;
}

InPort *StreamlineNode::datasetInput()
{
  return &m_mainDataInPort;
}

InPort *StreamlineNode::streamlineCoordsInput()
{
  return &m_coordsInPort;
}

bool StreamlineNode::isValid() const
{
  return m_mainDataInPort.isConnected() && m_coordsInPort.isConnected();
}

void StreamlineNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "steps"))
      m_steps = p->valueAs<int>();
    if (!std::strcmp(p->name(), "stepSize"))
      m_stepSize = p->valueAs<float>();
    if (!std::strcmp(p->name(), "calculateTime"))
      m_calculateTime = p->valueAs<bool>();
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

  auto retval = filter.Execute(ds);

  if (m_calculateTime)
    annotateIntegrationTime(retval, stepSize);

  return retval;
}

} // namespace graph
} // namespace vtkm_anari
