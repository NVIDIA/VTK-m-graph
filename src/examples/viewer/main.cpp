// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

// anari
#include <anari/anari_cpp.hpp>
// anari_viewer
#include <anari_viewer/Application.h>
#include <anari_viewer/windows/LightsEditor.h>
#include <anari_viewer/windows/Viewport.h>
// vtk-m
#include <vtkm/cont/Initialize.h>

#include "GraphControlsWindow.h"
#include "TransferFunctionEditor.h"

static bool g_verbose = false;
static std::string g_filename;

extern const char *getDefaultUILayout();

namespace viewer {

static void statusFunc(const void *userData,
    ANARIDevice device,
    ANARIObject source,
    ANARIDataType sourceType,
    ANARIStatusSeverity severity,
    ANARIStatusCode code,
    const char *message)
{
  const bool verbose = userData ? *(const bool *)userData : false;
  if (severity == ANARI_SEVERITY_FATAL_ERROR) {
    fprintf(stderr, "[FATAL][%p] %s\n", source, message);
    std::exit(1);
  } else if (severity == ANARI_SEVERITY_ERROR) {
    fprintf(stderr, "[ERROR][%p] %s\n", source, message);
  } else if (severity == ANARI_SEVERITY_WARNING) {
    fprintf(stderr, "[WARN ][%p] %s\n", source, message);
  } else if (verbose && severity == ANARI_SEVERITY_PERFORMANCE_WARNING) {
    fprintf(stderr, "[PERF ][%p] %s\n", source, message);
  } else if (verbose && severity == ANARI_SEVERITY_INFO) {
    fprintf(stderr, "[INFO ][%p] %s\n", source, message);
  } else if (verbose && severity == ANARI_SEVERITY_DEBUG) {
    fprintf(stderr, "[DEBUG][%p] %s\n", source, message);
  }
}

class Application : public anari_viewer::Application
{
 public:
  Application() = default;
  ~Application() override = default;

  anari_viewer::WindowArray setup() override
  {
    // ANARI //

    m_library = anari::loadLibrary("environment", statusFunc, &g_verbose);
    m_device = anari::newDevice(m_library, "default");

    if (!m_device)
      std::exit(1);

    // ImGui //

    ImGuiIO &io = ImGui::GetIO();
    io.FontGlobalScale = 2.f;
    io.IniFilename = nullptr;

    ImGui::LoadIniSettingsFromMemory(getDefaultUILayout());

    auto *controls = new vtkm3D::GraphControlsWindow(m_device, g_filename);
    auto *viewport = new windows::Viewport(m_device);
    auto *leditor = new windows::LightsEditor(m_device);
    auto *tfeditor = new windows::TransferFunctionEditor();

    auto world = controls->getANARIWorld();
    viewport->setWorld(world);
    leditor->setWorld(world);
    // tfeditor->setValueRange({g_voxelRange[0], g_voxelRange[1]});
    tfeditor->setUpdateCallback([=](const math::float2 &valueRange,
                                    const std::vector<math::float4> &co) {
      std::vector<math::float3> colors(co.size());
      std::vector<float> opacities(co.size());
      std::transform(
          co.begin(), co.end(), colors.begin(), [](const math::float4 &v) {
            return math::float3(v.x, v.y, v.z);
          });
      std::transform(
          co.begin(), co.end(), opacities.begin(), [](const math::float4 &v) {
            return v.w;
          });
      auto cArray = anari::newArray1D(m_device, colors.data(), colors.size());
      auto oArray =
          anari::newArray1D(m_device, opacities.data(), opacities.size());

      controls->setColorMapData(cArray, oArray, {valueRange.x, valueRange.y});

      anari::release(m_device, cArray);
      anari::release(m_device, oArray);
    });

    anari_viewer::WindowArray windows;
    windows.emplace_back(controls);
    windows.emplace_back(viewport);
    windows.emplace_back(leditor);
    windows.emplace_back(tfeditor);

    viewport->resetView();

    return windows;
  }

  void buildMainMenuUI() override
  {
    if (ImGui::BeginMainMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("print ImGui ini")) {
          const char *info = ImGui::SaveIniSettingsToMemory();
          printf("%s\n", info);
        }

        ImGui::EndMenu();
      }

      ImGui::EndMainMenuBar();
    }
  }

  void teardown() override
  {
    anari::release(m_device, m_device);
    anari::unloadLibrary(m_library);
  }

 private:
  anari::Library m_library{};
  anari::Device m_device{};
};

} // namespace viewer

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static void parseCommandLine(int argc, char *argv[])
{
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-v" || arg == "--verbose")
      g_verbose = true;
    else
      g_filename = arg;
  }
}

int main(int argc, char *argv[])
{
  vtkm::cont::Initialize(argc, argv, vtkm::cont::InitializeOptions::AddHelp);
  parseCommandLine(argc, argv);
  viewer::Application app;
  app.run(1920, 1080, "VTKm Graph App");
}
