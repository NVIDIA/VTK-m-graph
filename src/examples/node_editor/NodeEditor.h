// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// match3D
#include <match3D/match3D.h>
// vtkm_graph
#include "vtkm_graph/ExecutionGraph.h"
// anari_viewer
#include "anari_viewer/windows/Viewport.h"

#include "NodeInfoWindow.h"

namespace graph = vtkm::graph;

namespace nodes {

class NodeEditor : public match3D::Window
{
 public:
  NodeEditor(graph::ExecutionGraph *graph,
      windows::Viewport *viewport,
      vtkm3D::NodeInfoWindow *nodeInfoWindow);

  void buildUI() override;

 private:
  void contextMenu();
  void contextMenuPin();

  void updateWorld();
  void updateNodeSummary();

  void editor_Node(graph::Node *n);

  int m_summarizedNodeID{-1};
  int m_prevNumSelectedNodes{-1};
  int m_pinHoverId{-1};

  bool m_contextMenuVisible{false};
  bool m_contextPinMenuVisible{false};
  graph::ExecutionGraph *m_graph{nullptr};
  graph::TimeStamp m_lastGraphChange{};
  windows::Viewport *m_viewport{nullptr};
  vtkm3D::NodeInfoWindow *m_nodeInfoWindow{nullptr};
};

} // namespace nodes
