// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "FieldSelector.h"
#include "Node.h"
// vtk-m
#include <viskores/cont/DataSet.h>
#include <viskores/interop/anari/ANARIActor.h>

namespace viskores {
namespace graph {

struct VISKORES_GRAPH_EXPORT ActorNode : public Node
{
  ActorNode();
  ~ActorNode() override;

  const char *kind() const override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  NodeType type() const override;
  bool isValid() const override;

  const FieldSelector &cselector() const;
  FieldSelector &selector();

  void update() override;

 private:
  interop::anari::ANARIActor makeActor(cont::DataSet ds);

  InPort m_datasetPort{PortType::DATASET, "dataset", this};
  OutPort m_actorPort{PortType::ACTOR, "actor", this};
  FieldSelector m_selector;
};

} // namespace graph
} // namespace viskores
