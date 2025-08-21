// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "NodeInfoWindow.h"

namespace viskores3D {

NodeInfoWindow::NodeInfoWindow(anari_viewer::Application *app)
    : anari_viewer::windows::Window(app, "Node Info", true)
{}

void NodeInfoWindow::setText(std::string text)
{
  m_text = text;
}

void NodeInfoWindow::buildUI()
{
  ImGui::TextWrapped("%s", m_text.c_str());
}

} // namespace viskores3D
