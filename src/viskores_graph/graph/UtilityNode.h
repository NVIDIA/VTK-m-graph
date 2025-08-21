// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Node.h"
// vtk-m
#include <viskores/cont/DataSet.h>
// std
#include <functional>

namespace viskores {
namespace graph {

struct VISKORES_GRAPH_EXPORT UtilityNode : public Node
{
  UtilityNode(bool primary = false);
  ~UtilityNode() override = default;

  NodeType type() const override;
  bool isValid() const override;
};

// Concrete node types ////////////////////////////////////////////////////////

struct VISKORES_GRAPH_EXPORT DataSetToComponentsNode : public UtilityNode
{
  DataSetToComponentsNode();
  ~DataSetToComponentsNode();
  const char *kind() const override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  void update() override;

 private:
  InPort m_datasetInPort{PortType::DATASET, "dataset", this};
  std::vector<OutPort> m_outPorts;
};

struct VISKORES_GRAPH_EXPORT ComponentsToDataSetNode : public UtilityNode
{
  ComponentsToDataSetNode();
  ~ComponentsToDataSetNode();
  const char *kind() const override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  void update() override;

 private:
  std::vector<InPort> m_inPorts;
  OutPort m_datasetOutPort{PortType::DATASET, "dataset", this};
};

struct VISKORES_GRAPH_EXPORT VTKFileWriterNode : public UtilityNode
{
  VTKFileWriterNode();
  const char *kind() const override;

  void parameterChanged(Parameter *p, ParameterChangeType type) override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  void update() override;

 private:
  InPort m_datasetInPort{PortType::DATASET, "dataset", this};
};

using ValueRangeCallback = std::function<void(const Range &)>;

struct VISKORES_GRAPH_EXPORT ExtractActorFieldRangeNode : public UtilityNode
{
  ExtractActorFieldRangeNode();
  const char *kind() const override;

  void parameterChanged(Parameter *p, ParameterChangeType type) override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  void setCallback(ValueRangeCallback cb);

  bool needsUpdate() override;
  void update() override;

  Range getRange() const;

 private:
  InPort m_actorPort{PortType::ACTOR, "actor", this};
  ValueRangeCallback m_callback;
  bool m_active{true};
  Range m_range;
};

} // namespace graph
} // namespace viskores
