// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

// anari
#include <anari/anari_cpp.hpp>
// anari_viewer
#include <anari_viewer/Application.h>
#include <anari_viewer/windows/LightsEditor.h>
#include <anari_viewer/windows/Viewport.h>
// vtk-m
#include <vtkm/cont/Initialize.h>

#include "NodeEditor.h"

static bool g_verbose = false;

extern const char *getDefaultUILayout();

namespace nodes {

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

  anari_viewer::WindowArray setupWindows() override
  {
    // ANARI //

    m_library = anari::loadLibrary("environment", statusFunc, &g_verbose);
    m_device = anari::newDevice(m_library, "default");

    if (!m_device)
      std::exit(1);

    m_graph = std::make_unique<graph::ExecutionGraph>(m_device);

    // ImGui //

    ImNodes::CreateContext();

    ImNodesStyle &style = ImNodes::GetStyle();
    style.Colors[ImNodesCol_TitleBarHovered] = IM_COL32(160, 160, 160, 255);
    style.Colors[ImNodesCol_PinHovered] = IM_COL32(255, 255, 255, 255);
    style.PinOffset = style.PinCircleRadius / 4.f;
    style.PinCircleRadius *= 2.f;
    style.PinTriangleSideLength *= 2.f;
    style.PinQuadSideLength *= 2.f;
    style.PinHoverRadius *= 2.f;

    ImGuiIO &io = ImGui::GetIO();
    io.FontGlobalScale = 2.f;
    io.IniFilename = nullptr;

    ImGui::LoadIniSettingsFromMemory(getDefaultUILayout());

    auto *ninfo = new vtkm3D::NodeInfoWindow();
    auto *viewport = new windows::Viewport(m_device);
    auto *leditor = new windows::LightsEditor(m_device);
    auto *neditor = new NodeEditor(m_graph.get(), viewport, ninfo);

    auto world = m_graph->getANARIWorld();
    viewport->setWorld(world);
    leditor->setWorld(world);

    anari_viewer::WindowArray windows;
    windows.emplace_back(viewport);
    windows.emplace_back(leditor);
    windows.emplace_back(neditor);
    windows.emplace_back(ninfo);

    return windows;
  }

  void uiFrameStart() override
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
    ImNodes::DestroyContext();
  }

 private:
  anari::Library m_library{};
  anari::Device m_device{};

  std::unique_ptr<graph::ExecutionGraph> m_graph;
};

} // namespace nodes

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static void parseCommandLine(int argc, char *argv[])
{
  std::string filename;
  for (int i = 1; i < argc; i++) {
    std::string arg = argv[i];
    if (arg == "-v" || arg == "--verbose")
      g_verbose = true;
  }
}

int main(int argc, char *argv[])
{
  vtkm::cont::Initialize(argc, argv, vtkm::cont::InitializeOptions::AddHelp);
  parseCommandLine(argc, argv);
  nodes::Application app;
  app.run(1920, 1080, "VTKm Node Editor");
}
