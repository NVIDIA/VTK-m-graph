/*
 * Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// vtkm_anari
#include "vtkm_anari/ANARIMapperGlyphs.h"
#include "vtkm_anari/ANARIMapperVolume.h"
// vtk-m
#include <vtkm/filter/Gradient.h>
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
  }
  // omit ANARI_SEVERITY_INFO and ANARI_SEVERITY_DEBUG
}

static void setTF(anari::Device d, anari::Object obj)
{
  std::vector<glm::vec3> colors;
  std::vector<float> opacities;

  colors.emplace_back(0.f, 0.f, 1.f);
  colors.emplace_back(0.f, 1.f, 0.f);
  colors.emplace_back(1.f, 0.f, 0.f);

  opacities.emplace_back(0.f);
  opacities.emplace_back(1.f);

  anari::setAndReleaseParameter(
      d, obj, "color", anari::newArray1D(d, colors.data(), colors.size()));
  anari::setAndReleaseParameter(d,
      obj,
      "opacity",
      anari::newArray1D(d, opacities.data(), opacities.size()));
  anari::setParameter(d, obj, "valueRange", glm::vec2(0.f, 10.f));
}

static anari::Volume makeVolume(anari::Device d, anari::SpatialField field)
{
  if (!field)
    return nullptr;

  auto volume = anari::newObject<anari::Volume>(d, "scivis");
  anari::setParameter(d, volume, "field", field);
  anari::setParameter(d, volume, "densityScale", 0.05f);

  setTF(d, volume);

  anari::commit(d, volume);

  return volume;
}

static anari::Surface makeSurface(anari::Device d, anari::Geometry geometry)
{
  if (!geometry)
    return nullptr;

  auto material = anari::newObject<anari::Material>(d, "matte");
  anari::setParameter(d, material, "color", glm::vec4(1.f));
  anari::commit(d, material);

  auto surface = anari::newObject<anari::Surface>(d);
  anari::setParameter(d, surface, "geometry", geometry);
  anari::setAndReleaseParameter(d, surface, "material", material);
  anari::commit(d, surface);

  return surface;
}

int main()
{
  stbi_flip_vertically_on_write(1);

  {
    // Initialize ANARI ///////////////////////////////////////////////////////

    printf("initialize ANARI...");

    auto lib = anari::loadLibrary("visrtx", anariStatusFunc);
    auto d = anari::newDevice(lib, "default");

    printf("done\n");

    // Create VTKm datasets ///////////////////////////////////////////////////

    printf("generating 'tangle' volume...");

    auto tangle = vtkm::source::Tangle(vtkm::Id3{64}).Execute();

    printf("done\n");
    printf("generating 'tangle' isosurface...");

    auto &tangle_field = tangle.GetField(0);
    vtkm::Range range;
    tangle_field.GetRange(&range);
    const auto isovalue = range.Center();

    vtkm::filter::Gradient gradientFilter;
    gradientFilter.SetActiveField(tangle_field.GetName());
    gradientFilter.SetOutputFieldName("Gradient");
    auto tangleGrad = gradientFilter.Execute(tangle);

    printf("done\n");

    // Map data to ANARI objects //////////////////////////////////////////////

    printf("mapping VTKm datasets to anari::World...");

    auto world = anari::newObject<anari::World>(d);

    vtkm_anari::ANARIActor va(
        tangle.GetCellSet(), tangle.GetCoordinateSystem(), tangle_field);

    vtkm_anari::ANARIMapperVolume mVol(d, va);
    anari::Volume v = makeVolume(d, mVol.GetANARISpatialField());

    if (v) {
      anari::setAndReleaseParameter(
          d, world, "volume", anari::newArray1D(d, &v));
      anari::release(d, v);
    }

    vtkm_anari::ANARIActor sa(tangleGrad.GetCellSet(),
        tangleGrad.GetCoordinateSystem(),
        tangleGrad.GetField("Gradient"));

    vtkm_anari::ANARIMapperGlyphs mGlyphs(d, sa);
    anari::Surface s = makeSurface(d, mGlyphs.GetANARIGeometry());

    if (s) {
      anari::setAndReleaseParameter(
          d, world, "surface", anari::newArray1D(d, &s));
      anari::release(d, s);
    }

    anari::commit(d, world);

    printf("done\n");

    // Render a frame /////////////////////////////////////////////////////////

    printf("creating anari::Frame and rendering it...");

    auto renderer = anari::newObject<anari::Renderer>(d, "raycast");
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
    stbi_write_png("glyph.png", 1024, 768, 4, fb, 4 * 1024);
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