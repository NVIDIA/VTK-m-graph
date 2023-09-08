// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/flow/Streamline.h>
#include <vtkm/worklet/WorkletMapField.h>
#include <vtkm/worklet/WorkletMapTopology.h>
// std
#include <cmath>
#include <cstring>

namespace vtkm {
namespace graph {

// Worklets ///////////////////////////////////////////////////////////////////

class CoordinateSystemToParticles : public worklet::WorkletMapField
{
 public:
  VTKM_CONT CoordinateSystemToParticles() = default;

  using ControlSignature = void(FieldIn, FieldOut);
  using ExecutionSignature = void(InputIndex, _1, _2);

  VTKM_EXEC void operator()(const Id out_idx,
      const Vec3f_32 &in_coord,
      Particle &outP) const
  {
    outP = Particle(in_coord, out_idx);
  }
};

class CalculateIntegrationTime
    : public worklet::WorkletVisitCellsWithPoints
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
      const IdComponent &numPoints,
      const InIndicesType &indices,
      OutTimePortalType &outTime) const
  {
    for (int i = 0; i < numPoints; ++i) {
      outTime.Set(indices[i],
          shapeType.Id == CELL_SHAPE_POLY_LINE ? i * StepSize : 0.f);
    }
  }
};

// Helper functions ///////////////////////////////////////////////////////////

static void annotateIntegrationTime(cont::DataSet &ds, float stepSize)
{
  if (!ds.GetCellSet().IsType<cont::CellSetExplicit<>>())
    return;

  auto cells = ds.GetCellSet().AsCellSet<cont::CellSetExplicit<>>();

  cont::ArrayHandle<float> integrationTimeFieldArray;
  integrationTimeFieldArray.Allocate(ds.GetNumberOfPoints());

  CalculateIntegrationTime worklet(stepSize);
  worklet::DispatcherMapTopology<CalculateIntegrationTime>(worklet)
      .Invoke(cells, integrationTimeFieldArray);

  ds.AddField(cont::make_FieldPoint(
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

cont::DataSet StreamlineNode::execute()
{
  if (!datasetInput()->isConnected() || !streamlineCoordsInput()->isConnected())
    return {};

  auto ds = getDataSetFromPort(datasetInput());
  auto cds = getDataSetFromPort(streamlineCoordsInput());

  float stepSize = m_stepSize;
  auto coords = cds.GetCoordinateSystem();

  if (stepSize == 0.f) {
    const Bounds bounds = coords.GetBounds();
    const float diagonal =
        std::sqrt(std::pow((float)(bounds.X.Max - bounds.X.Min), 2.f)
            + std::pow((float)(bounds.Y.Max - bounds.Y.Min), 2.f)
            + std::pow((float)(bounds.Z.Max - bounds.Z.Min), 2.f));
    stepSize = diagonal / 1000.f;
  }

  cont::ArrayHandle<Particle> seedArray;
  seedArray.Allocate(coords.GetNumberOfValues());
  CoordinateSystemToParticles worklet;
  worklet::DispatcherMapField<CoordinateSystemToParticles>(worklet)
      .Invoke(coords, seedArray);

  filter::flow::Streamline filter;
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
} // namespace vtkm
