// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "GraphControlsWindow.h"

namespace vtkm3D {

// Helper functions ///////////////////////////////////////////////////////////

static void ui_NodeParameter(graph::ExecutionGraph *g, graph::Parameter &p)
{
  if (p.type() == graph::ParameterType::BOOL) {
    auto value = p.valueAs<bool>();
    if (ImGui::Checkbox(p.name(), &value))
      g->scheduleParameterUpdate(p, value);
  } else if (p.type() == graph::ParameterType::BOUNDED_FLOAT) {
    ImGui::BeginDisabled(!p.hasMinMax());
    auto value = p.valueAs<float>();
    if (ImGui::SliderFloat(
            p.name(), &value, p.minAs<float>(), p.maxAs<float>()))
      g->scheduleParameterUpdate(p, value);
    ImGui::EndDisabled();
  } else if (p.type() == graph::ParameterType::BOUNDED_INT) {
    ImGui::BeginDisabled(!p.hasMinMax());
    auto value = p.valueAs<int>();
    if (ImGui::SliderInt(p.name(), &value, p.minAs<int>(), p.maxAs<int>()))
      g->scheduleParameterUpdate(p, value);
    ImGui::EndDisabled();
  } else if (p.type() == graph::ParameterType::FLOAT) {
    auto value = p.valueAs<float>();
    if (ImGui::InputFloat(p.name(), &value))
      g->scheduleParameterUpdate(p, value);
  } else if (p.type() == graph::ParameterType::INT) {
    auto value = p.valueAs<int>();
    if (ImGui::InputInt(p.name(), &value))
      g->scheduleParameterUpdate(p, value);
  } else if (p.type() == graph::ParameterType::FILENAME) {
    auto value = p.valueAs<std::string>();
    std::vector<char> buf(value.size() + 1 + 256);
    strcpy(buf.data(), value.c_str());
    buf[value.size()] = '\0';
    if (ImGui::InputText(p.name(),
            buf.data(),
            value.size() + 256,
            ImGuiInputTextFlags_EnterReturnsTrue)) {
      g->scheduleParameterUpdate(p, std::string(buf.data()));
    }
  } else
    ImGui::Text("param: %s", p.name());
}

static void ui_NodeParameters(graph::ExecutionGraph *g, graph::Node *n)
{
  ImGui::Text("%s:", n->name());
  ImGui::PushID(n);
  std::for_each(n->parametersBegin(), n->parametersEnd(), [&](auto &p) {
    ui_NodeParameter(g, p);
  });
  ImGui::PopID();
}

// GraphControlsWindow definitions ////////////////////////////////////////////

GraphControlsWindow::GraphControlsWindow(anari::Device d, const std::string &f)
    : anari_viewer::Window("Graph Controls", true), m_graph(d)
{
  if (!f.empty()) {
    m_nodes.source = m_graph.addNode<graph::VTKFileReaderNode>();
    m_nodes.source->parameter("filename")->setValue(f);
  }
  else
    m_nodes.source = m_graph.addNode<graph::TangleSourceNode>();

  m_nodes.contour = m_graph.addNode<graph::ContourNode>();
  m_nodes.actor1 = m_graph.addNode<graph::ActorNode>();
  m_nodes.actor2 = m_graph.addNode<graph::ActorNode>();
  m_nodes.volumeMapper = m_graph.addNode<graph::VolumeMapperNode>();
  m_nodes.triangleMapper = m_graph.addNode<graph::TriangleMapperNode>();

  m_nodes.volumeMapper->parameter("visible")->setValue(false);

  graph::connect(
      m_nodes.source->output("dataset"), m_nodes.actor1->input("dataset"));
  graph::connect(
      m_nodes.actor1->output("actor"), m_nodes.volumeMapper->input("actor"));

  graph::connect(
      m_nodes.source->output("dataset"), m_nodes.contour->input("dataset"));
  graph::connect(
      m_nodes.contour->output("dataset"), m_nodes.actor2->input("dataset"));
  graph::connect(
      m_nodes.actor2->output("actor"), m_nodes.triangleMapper->input("actor"));

  m_graph.update();
  m_graph.sync();
}

anari::World GraphControlsWindow::getANARIWorld() const
{
  return m_graph.getANARIWorld();
}

void GraphControlsWindow::buildUI()
{
  ui_NodeParameters(&m_graph, m_nodes.source);
  ImGui::Separator();
  ui_NodeParameters(&m_graph, m_nodes.contour);
  ImGui::Separator();
  ui_NodeParameters(&m_graph, m_nodes.volumeMapper);
  ImGui::Separator();
  ui_NodeParameters(&m_graph, m_nodes.triangleMapper);
  m_graph.update();
}

} // namespace vtkm3D
