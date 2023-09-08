// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

// anari
#include <anari/anari_cpp/ext/glm.h>
#include <anari/anari_cpp.hpp>
// vtk-m
#include <vtkm/cont/Initialize.h>
// stb
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "vtkm_graph/ExecutionGraph.h"

static void anariStatusFunc(const void *,
    ANARIDevice,
    ANARIObject,
    ANARIDataType,
    ANARIStatusSeverity severity,
    ANARIStatusCode,
    const char *message)
{
  if (severity == ANARI_SEVERITY_FATAL_ERROR) {
    fprintf(stderr, "[FATAL] %s\n", message);
    std::exit(1);
  } else if (severity == ANARI_SEVERITY_ERROR) {
    fprintf(stderr, "[ERROR] %s\n", message);
  } else if (severity == ANARI_SEVERITY_WARNING) {
    fprintf(stderr, "[WARN ] %s\n", message);
  } else if (severity == ANARI_SEVERITY_PERFORMANCE_WARNING) {
    fprintf(stderr, "[PERF ] %s\n", message);
#if 1
  } // omit ANARI_SEVERITY_INFO and ANARI_SEVERITY_DEBUG
#else
  } else if (severity == ANARI_SEVERITY_INFO) {
    fprintf(stderr, "[INFO ] %s\n", message);
  } else if (severity == ANARI_SEVERITY_DEBUG) {
    fprintf(stderr, "[DEBUG] %s\n", message);
  }
#endif
}

template <typename... Args>
static void print_status(const char *msg, Args &&...args)
{
  printf(msg, std::forward<Args>(args)...);
  fflush(stdout);
}

namespace graph = vtkm::graph;

int main(int argc, char *argv[])
{
  stbi_flip_vertically_on_write(1);

  {
    // Initialize VTK-m //

    vtkm::cont::Initialize(argc, argv);

    // Initialize ANARI //

    print_status("initialize ANARI...");

    auto lib = anari::loadLibrary("environment", anariStatusFunc);
    auto d = anari::newDevice(lib, "default");

    print_status("done\n");

    // Build execution graph //

    graph::ExecutionGraph graph(d);

    print_status("creating execution graph...");

    // Make nodes //

    auto *tangleNode = graph.addNode<graph::TangleSourceNode>();
    auto *contourNode = graph.addNode<graph::ContourNode>();
    auto *actorNode1 = graph.addNode<graph::ActorNode>();
    auto *actorNode2 = graph.addNode<graph::ActorNode>();
    auto *mapVolumeNode = graph.addNode<graph::VolumeMapperNode>();
    auto *mapTriNode = graph.addNode<graph::TriangleMapperNode>();

    // Connect nodes //

    // Volume
    graph::connect(tangleNode->output("dataset"), actorNode1->input("dataset"));
    graph::connect(actorNode1->output("actor"), mapVolumeNode->input("actor"));

    // Isosurface
    graph::connect(
        tangleNode->output("dataset"), contourNode->input("dataset"));
    graph::connect(
        contourNode->output("dataset"), actorNode2->input("dataset"));
    graph::connect(actorNode2->output("actor"), mapTriNode->input("actor"));

    print_status("done\n");

    // Final update of the graph to populate the world //

    print_status("updating graph...");

    auto frame = anari::newObject<anari::Frame>(d);

#if 1
    graph.update([&]() {
      print_status("creating anari::Frame and rendering it...");

      auto renderer = anari::newObject<anari::Renderer>(d, "default");
      anari::setParameter(
          d, renderer, "background", glm::vec4(0.f, 0.f, 0.f, 1.f));
      anari::commitParameters(d, renderer);

      auto camera = anari::newObject<anari::Camera>(d, "perspective");
      anari::setParameter(d, camera, "aspect", 1024 / 768.f);
      anari::setParameter(d, camera, "position", glm::vec3(-0.05, 1.43, 1.87));
      anari::setParameter(
          d, camera, "direction", glm::vec3(0.32, -0.53, -0.79));
      anari::setParameter(d, camera, "up", glm::vec3(-0.20, -0.85, 0.49));
      anari::commitParameters(d, camera);

      anari::setParameter(d, frame, "size", glm::uvec2(1024, 768));
      anari::setParameter(d, frame, "channel.color", ANARI_UFIXED8_RGBA_SRGB);
      anari::setParameter(d, frame, "world", graph.getANARIWorld());
      anari::setAndReleaseParameter(d, frame, "camera", camera);
      anari::setAndReleaseParameter(d, frame, "renderer", renderer);
      anari::commitParameters(d, frame);

      anari::render(d, frame);
      anari::wait(d, frame);

      const auto fb = anari::map<uint32_t>(d, frame, "channel.color");
      stbi_write_png("graph.png",
          int(fb.width),
          int(fb.height),
          4,
          fb.data,
          4 * int(fb.width));
      anari::unmap(d, frame, "channel.color");

      print_status("done\n");
    });
#else
    graph.update();
#endif
    if (!graph.isReady())
      print_status("(its async!)...");
    graph.sync();

    print_status("done\n");

    graph.print();

    // Render a frame //

#if 0
    print_status("creating anari::Frame and rendering it...");

    auto renderer = anari::newObject<anari::Renderer>(d, "default");
    anari::setParameter(
        d, renderer, "background", glm::vec4(0.f, 0.f, 0.f, 1.f));
    anari::commitParameters(d, renderer);

    auto camera = anari::newObject<anari::Camera>(d, "perspective");
    anari::setParameter(d, camera, "aspect", 1024 / 768.f);
    anari::setParameter(d, camera, "position", glm::vec3(-0.05, 1.43, 1.87));
    anari::setParameter(d, camera, "direction", glm::vec3(0.32, -0.53, -0.79));
    anari::setParameter(d, camera, "up", glm::vec3(-0.20, -0.85, 0.49));
    anari::commitParameters(d, camera);

    auto frame = anari::newObject<anari::Frame>(d);
    anari::setParameter(d, frame, "size", glm::uvec2(1024, 768));
    anari::setParameter(d, frame, "channel.color", ANARI_UFIXED8_RGBA_SRGB);
    anari::setParameter(d, frame, "world", graph.getANARIWorld());
    anari::setAndReleaseParameter(d, frame, "camera", camera);
    anari::setAndReleaseParameter(d, frame, "renderer", renderer);
    anari::commitParameters(d, frame);

    anari::render(d, frame);
    anari::wait(d, frame);

    const auto fb = anari::map<uint32_t>(d, frame, "channel.color");
    stbi_write_png("graph.png",
        int(fb.width),
        int(fb.height),
        4,
        fb.data,
        4 * int(fb.width));
    anari::unmap(d, frame, "channel.color");

    print_status("done\n");
#endif

    // Cleanup //

    print_status("cleaning up remaining objects...");

    anari::release(d, frame);
    anari::release(d, d);
    anari::unloadLibrary(lib);

    print_status("done\n");
  }

  return 0;
}
