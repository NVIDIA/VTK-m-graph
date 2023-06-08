// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// match3D
#include <match3D/match3D.h>

namespace vtkm3D {

class NodeInfoWindow : public match3D::Window
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
