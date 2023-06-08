// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// match3D
#include <match3D/match3D.h>
// vtkm_graph
#include "vtkm_graph/ExecutionGraph.h"

namespace graph = vtkm::graph;

namespace vtkm3D {

class GraphControlsWindow : public match3D::Window
{
 public:
  GraphControlsWindow(anari::Device d);
  ~GraphControlsWindow() = default;

  void buildUI() override;

  anari::World getANARIWorld() const;

 private:
  graph::ExecutionGraph m_graph;

  struct Nodes {
  graph::TangleSourceNode *tangle{nullptr};
  graph::ContourNode *contour{nullptr};
  graph::ActorNode *actor1{nullptr};
  graph::ActorNode *actor2{nullptr};
  graph::VolumeMapperNode *volumeMapper{nullptr};
  graph::TriangleMapperNode *triangleMapper{nullptr};
  } m_nodes;
};

} // namespace vtkm3D
