// Copyright 2022 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

// vtkm_anari
#include "vtkm_anari/ANARIMapper.h"
// vtk-m
#include <vtkm/filter/Contour.h>
#include <vtkm/source/Tangle.h>
// stb
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
// anari
#include <anari/anari_cpp/ext/glm.h>
// std
#include <vector>

static void anariStatusFunc(void *,
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
  } else if (severity == ANARI_SEVERITY_INFO) {
    fprintf(stderr, "[INFO ] %s\n", message);
  } else if (severity == ANARI_SEVERITY_DEBUG) {
    fprintf(stderr, "[DEBUG] %s\n", message);
  }
}

int main()
{
  stbi_flip_vertically_on_write(1);

  {
    // Initialize ANARI ///////////////////////////////////////////////////////

    printf("initialize ANARI...\n");

    auto lib = anari::loadLibrary("example", anariStatusFunc);
    auto d = anari::newDevice(lib, "default");

    // Create VTKm datasets ///////////////////////////////////////////////////

    printf("generating 'tangle' volume...");

    auto tangle = vtkm::source::Tangle(vtkm::Id3{64}).Execute();

    printf("done\n");
    printf("generating 'tangle' isosurface...");

    auto &tangle_field = tangle.GetField(0);
    vtkm::Range range;
    tangle_field.GetRange(&range);
    const auto isovalue = range.Center();

    vtkm::filter::Contour filter;
    filter.SetIsoValue(isovalue);
    filter.SetActiveField(tangle_field.GetName());
    auto tangleIso = filter.Execute(tangle);

    printf("done\n");

    // Map data to ANARI objects //////////////////////////////////////////////

    printf("mapping VTKm datasets to anari::World...");

    auto world = anari::newObject<anari::World>(d);

    std::vector<anari::Surface> surfaces;
    std::vector<anari::Volume> volumes;

    auto applyScene = [&](auto &&arg) {
      using T = std::decay_t<decltype(arg)>;
      if constexpr (std::is_same_v<T, anari::Surface>) {
        if (arg)
          surfaces.push_back(arg);
      } else if constexpr (std::is_same_v<T, anari::Volume>) {
        if (arg)
          volumes.push_back(arg);
      }
    };

    vtkm_anari::Actor va = {tangle, vtkm_anari::Representation::VOLUME, 0};
    std::visit(applyScene, vtkm_anari::makeANARIObject(d, va));

    vtkm_anari::Actor sa = {
        tangleIso, vtkm_anari::Representation::TRIANGLES, 0};
    std::visit(applyScene, vtkm_anari::makeANARIObject(d, sa));

    if (!volumes.empty()) {
      anari::setAndReleaseParameter(d,
          world,
          "volume",
          anari::newArray1D(d, volumes.data(), volumes.size()));
    }

    for (auto v : volumes)
      anari::release(d, v);

    if (!surfaces.empty()) {
      anari::setAndReleaseParameter(d,
          world,
          "surface",
          anari::newArray1D(d, surfaces.data(), surfaces.size()));
    }

    for (auto s : surfaces)
      anari::release(d, s);

    anari::commit(d, world);

    printf("done\n");

    // Render a frame /////////////////////////////////////////////////////////

    printf("creating anari::Frame and rendering it...");

    auto renderer = anari::newObject<anari::Renderer>(d, "default");
    anari::setParameter(
        d, renderer, "backgroundColor", glm::vec4(0.f, 0.f, 0.f, 1.f));
    anari::commit(d, renderer);

    auto camera = anari::newObject<anari::Camera>(d, "perspective");
    anari::setParameter(d, camera, "aspect", 1024 / float(768));
    anari::setParameter(d, camera, "position", glm::vec3(-0.05, 1.43, 1.87));
    anari::setParameter(d, camera, "direction", glm::vec3(0.32, -0.53, -0.79));
    anari::setParameter(d, camera, "up", glm::vec3(-0.20, -0.85, 0.49));
    anari::commit(d, camera);

    auto frame = anari::newObject<anari::Frame>(d);
    anari::setParameter(d, frame, "size", glm::uvec2(1024, 768));
    anari::setParameter(d, frame, "color", ANARI_UFIXED8_RGBA_SRGB);
    anari::setParameter(d, frame, "world", world);
    anari::setParameter(d, frame, "camera", camera);
    anari::setParameter(d, frame, "renderer", renderer);
    anari::commit(d, frame);

    anari::release(d, camera);
    anari::release(d, renderer);
    anari::release(d, world);

    anari::render(d, frame);
    anari::wait(d, frame);

    const uint32_t *fb = (uint32_t *)anari::map(d, frame, "color");
    stbi_write_png("example.png", 1024, 768, 4, fb, 4 * 1024);
    anari::unmap(d, frame, "color");

    printf("done\n");

    // Cleanup ////////////////////////////////////////////////////////////////

    printf("cleaning up remaining objects...");

    anari::release(d, frame);
    anari::release(d, d);
    anari::unloadLibrary(lib);

    printf("done\n");
  }

  return 0;
}