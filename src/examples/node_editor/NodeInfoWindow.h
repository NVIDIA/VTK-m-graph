// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// anari_viewer
#include <anari_viewer/windows/Window.h>

namespace vtkm3D {

class NodeInfoWindow : public anari_viewer::Window
{
 public:
  NodeInfoWindow();
  ~NodeInfoWindow() = default;

  void setText(std::string text);

  void buildUI() override;

 private:
  std::string m_text{"<no summary>"};
};

} // namespace vtkm3D
