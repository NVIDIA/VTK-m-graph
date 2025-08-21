// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Node.h"
// vtk-m
#include <viskores/cont/DataSet.h>

namespace viskores {
namespace graph {

struct VISKORES_GRAPH_EXPORT FilterNode : public Node
{
  FilterNode() = default;
  ~FilterNode() override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  virtual InPort *datasetInput();

  NodeType type() const override;
  bool isValid() const override;

  void update() override;

 private:
  virtual bool needsUpdate() override;
  virtual cont::DataSet execute() = 0;

  OutPort m_datasetOutPort{PortType::DATASET, "dataset", this};
  InPort m_datasetInPort{PortType::DATASET, "dataset", this};
};

// Concrete node types ////////////////////////////////////////////////////////

struct VISKORES_GRAPH_EXPORT CellAverageNode : public FilterNode
{
  CellAverageNode() = default;
  const char *kind() const override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT CleanGridNode : public FilterNode
{
  CleanGridNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;

  bool m_compactPointFields{true};
  bool m_mergePoints{true};
  bool m_fastMerge{false};
  bool m_removeDegenerateCells{true};
};

struct VISKORES_GRAPH_EXPORT ContourNode : public FilterNode
{
  ContourNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT GradientNode : public FilterNode
{
  GradientNode() = default;
  const char *kind() const override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT PointAverageNode : public FilterNode
{
  PointAverageNode() = default;
  const char *kind() const override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT ProbeNode : public FilterNode
{
  ProbeNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

  InPort *inputBegin() override;
  size_t numInput() const override;

  InPort *datasetInput() override;
  InPort *sampleQuantityInput();

  bool isValid() const override;

 private:
  cont::DataSet execute() override;

  InPort m_mainDataInPort{PortType::DATASET, "dataset", this};
  InPort m_sampleQuantityInPort{PortType::DATASET, "sample quantity", this};
  bool m_removeHiddenFields{true};
};

struct VISKORES_GRAPH_EXPORT SliceNode : public FilterNode
{
  SliceNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;

  Vec3f m_azelpos{0.f, 0.f, 0.5f};
};

struct VISKORES_GRAPH_EXPORT StreamlineNode : public FilterNode
{
  StreamlineNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

  InPort *inputBegin() override;
  size_t numInput() const override;

  InPort *datasetInput() override;
  InPort *streamlineCoordsInput();

  bool isValid() const override;

 private:
  cont::DataSet execute() override;

  InPort m_mainDataInPort{PortType::DATASET, "dataset", this};
  InPort m_coordsInPort{PortType::DATASET, "streamline coords", this};

  int m_steps{100};
  float m_stepSize{0.f};
  bool m_calculateTime{true};
};

struct VISKORES_GRAPH_EXPORT SurfaceNormalsNode : public FilterNode
{
  SurfaceNormalsNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;

  bool m_oriented{true};
  bool m_flip{true};
};

struct VISKORES_GRAPH_EXPORT TetrahedralizeNode : public FilterNode
{
  TetrahedralizeNode() = default;
  const char *kind() const override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT TubeNode : public FilterNode
{
  TubeNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;

  int m_sides{7};
  float m_radius{0.f};
  bool m_cap{false};
};

struct VISKORES_GRAPH_EXPORT VectorMagnitudeNode : public FilterNode
{
  VectorMagnitudeNode() = default;
  const char *kind() const override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT VertexClusteringNode : public FilterNode
{
  VertexClusteringNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;
};

} // namespace graph
} // namespace viskores
