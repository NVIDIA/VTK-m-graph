// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// glad
#include <glad/glad.h>
// anari
#include <anari/anari_cpp/ext/linalg.h>
#include <anari_viewer/windows/Window.h>
// std
#include <functional>
#include <string>
#include <vector>

namespace math = anari::math;

namespace windows {

using ColorPoint = math::float4;
using OpacityPoint = math::float2;

using TFUpdateCallback = std::function<void(
    const math::float2 &, const std::vector<math::float4> &)>;

class TransferFunctionEditor : public anari_viewer::Window
{
 public:
  TransferFunctionEditor(const char *name = "TF Editor");
  ~TransferFunctionEditor();

  void buildUI() override;

  void setUpdateCallback(TFUpdateCallback cb);
  void triggerUpdateCallback();

  void setValueRange(const math::float2 &vr);

  // getters for current transfer function data
  math::float2 getValueRange();
  std::vector<math::float4> getSampledColorsAndOpacities(int numSamples = 256);

 private:
  void loadDefaultMaps();
  void setMap(int);

  math::float3 interpolateColor(
      const std::vector<ColorPoint> &controlPoints, float x);

  float interpolateOpacity(
      const std::vector<OpacityPoint> &controlPoints, float x);

  void updateTfnPaletteTexture();

  void drawEditor();

  // callback called whenever transfer function is updated
  TFUpdateCallback m_updateCallback;

  // all available transfer functions
  std::vector<std::string> m_tfnsNames;
  std::vector<std::vector<ColorPoint>> m_tfnsColorPoints;
  std::vector<std::vector<OpacityPoint>> m_tfnsOpacityPoints;
  std::vector<bool> m_tfnsEditable;

  // parameters of currently selected transfer function
  int m_currentMap{0};
  std::vector<ColorPoint> *m_tfnColorPoints{nullptr};
  std::vector<OpacityPoint> *m_tfnOpacityPoints{nullptr};
  bool m_tfnEditable{true};

  // flag indicating transfer function has changed in UI
  bool m_tfnChanged{true};

  // scaling factor for generated opacities
  float m_globalOpacityScale{1.f};

  // domain (value range) of transfer function
  math::float2 m_valueRange{-1.f, 1.f};
  math::float2 m_defaultValueRange{-1.f, 1.f};

  // texture for displaying transfer function color palette
  GLuint tfnPaletteTexture{0};
};

} // namespace windows
