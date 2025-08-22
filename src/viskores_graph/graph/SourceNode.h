// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Node.h"
// viskores
#include <viskores/cont/DataSet.h>
// std
#include <functional>

namespace viskores {
namespace graph {

struct VISKORES_GRAPH_EXPORT SourceNode : public Node
{
  SourceNode() = default;
  ~SourceNode() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  NodeType type() const override;
  bool isValid() const override;

  void update() override;

 private:
  virtual cont::DataSet execute() = 0;

  OutPort m_datasetPort{PortType::DATASET, "dataset", this};
};

// Concrete node types ////////////////////////////////////////////////////////

struct VISKORES_GRAPH_EXPORT ABCSourceNode : public SourceNode
{
  ABCSourceNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;
};

using SourceNodeCallback = std::function<cont::DataSet()>;

struct VISKORES_GRAPH_EXPORT CallbackSourceNode : public SourceNode
{
  CallbackSourceNode() = default;
  CallbackSourceNode(SourceNodeCallback cb);
  const char *kind() const override;

  void setCallback(SourceNodeCallback cb);

  void markChanged(); // Application signal to refresh dataset via callback

 private:
  cont::DataSet execute() override;

  SourceNodeCallback m_callback = []() -> cont::DataSet { return {}; };
};

struct VISKORES_GRAPH_EXPORT RandomPointsSourceNode : public SourceNode
{
  RandomPointsSourceNode() = default;
  const char *kind() const override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT TangleSourceNode : public SourceNode
{
  TangleSourceNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;
};

struct VISKORES_GRAPH_EXPORT VTKFileReaderNode : public SourceNode
{
  VTKFileReaderNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  cont::DataSet execute() override;
};

} // namespace graph
} // namespace viskores
