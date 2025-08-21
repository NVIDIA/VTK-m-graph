// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// anari_viewer
#include <anari_viewer/windows/Viewport.h>
#include <anari_viewer/windows/Window.h>
// imnodes
#include <imnodes.h>
// viskores_graph
#include "viskores_graph/ExecutionGraph.h"
// anari_viewer

#include "NodeInfoWindow.h"

namespace graph = viskores::graph;

namespace nodes {

class NodeEditor : public anari_viewer::windows::Window
{
 public:
  NodeEditor(anari_viewer::Application *app,
      graph::ExecutionGraph *graph,
      anari_viewer::windows::Viewport *viewport,
      viskores3D::NodeInfoWindow *nodeInfoWindow);

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
  anari_viewer::windows::Viewport *m_viewport{nullptr};
  viskores3D::NodeInfoWindow *m_nodeInfoWindow{nullptr};
};

} // namespace nodes
