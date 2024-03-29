// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// anari_viewer
#include <anari_viewer/windows/Window.h>
// vtkm_graph
#include "vtkm_graph/ExecutionGraph.h"

namespace graph = vtkm::graph;

namespace vtkm3D {

class GraphControlsWindow : public anari_viewer::Window
{
 public:
  GraphControlsWindow(anari::Device d, const std::string &filename = "");
  ~GraphControlsWindow() = default;

  void buildUI() override;

  anari::World getANARIWorld() const;
  void setColorMapData(anari_cpp::Array1D color,
      anari_cpp::Array1D opacity,
      const vtkm::Vec2f_32 &valueRange);

  vtkm::Range getDataRange() const;

 private:
  graph::ExecutionGraph m_graph;

  struct Nodes
  {
    graph::SourceNode *source{nullptr};
    graph::ContourNode *contour{nullptr};
    graph::ActorNode *actor1{nullptr};
    graph::ActorNode *actor2{nullptr};
    graph::VolumeMapperNode *volumeMapper{nullptr};
    graph::TriangleMapperNode *triangleMapper{nullptr};
    graph::ExtractActorFieldRangeNode *valueRangeNode{nullptr};
  } m_nodes;
};

} // namespace vtkm3D
