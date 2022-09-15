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

// benchmark
#include <benchmark/benchmark.h>
// vtkm_anari
#include "vtkm_anari/ANARIMapperPoints.h"
#include "vtkm_anari/ANARIMapperTriangles.h"
#include "vtkm_anari/ANARIScene.h"
// vtk-m
#include <vtkm/filter/contour/Contour.h>
#include <vtkm/source/Tangle.h>
// stb
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
// anari
#include <anari/anari_cpp/ext/glm.h>
// std
#include <vector>

anari::Library lib = nullptr;
anari::Device d = nullptr;

// Helper functions ///////////////////////////////////////////////////////////

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
  }
}

static void setTF(anari::Device d, vtkm_anari::ANARIMapper &mapper)
{
  auto colorArray = anari::newArray1D(d, ANARI_FLOAT32_VEC3, 3);
  auto *colors = anari::map<glm::vec3>(d, colorArray);
  colors[0] = glm::vec3(0.f, 0.f, 1.f);
  colors[1] = glm::vec3(0.f, 1.f, 0.f);
  colors[2] = glm::vec3(1.f, 0.f, 0.f);
  anari::unmap(d, colorArray);

  auto opacityArray = anari::newArray1D(d, ANARI_FLOAT32, 2);
  auto *opacities = anari::map<float>(d, opacityArray);
  opacities[0] = 0.f;
  opacities[1] = 1.f;
  anari::unmap(d, opacityArray);

  mapper.SetANARIColorMapArrays(
      colorArray, nullptr, opacityArray, nullptr, true);
  mapper.SetANARIColorMapValueRange(vtkm::Vec2f_32(0.f, 10.f));
  mapper.SetANARIColorMapOpacityScale(0.05f);
}

static vtkm::cont::DataSet makeIsosurface(
    vtkm::cont::DataSet ds, float normalizedIsovalue)
{
  auto &field = ds.GetField(0);
  vtkm::Range range;
  field.GetRange(&range);

  const float isovalue = normalizedIsovalue * range.Length() + range.Min;

  vtkm::filter::contour::Contour contourFilter;
  contourFilter.SetIsoValue(isovalue);
  contourFilter.SetActiveField(field.GetName());
  return contourFilter.Execute(ds);
}

///////////////////////////////////////////////////////////////////////////////
// Benchmarks /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <bool TRIANGLES>
static void bench_isosurface(benchmark::State &state)
{
  // Create VTKm datasets //

  const auto size = state.range(0);
  auto tangle = vtkm::source::Tangle(vtkm::Id3{size}).Execute();
  auto tangleIso = makeIsosurface(tangle, 0.25f);

  // Map data to ANARI objects //

  vtkm_anari::ANARIScene scene(d);

  if (TRIANGLES) {
    auto &mIso = scene.AddMapper(vtkm_anari::ANARIMapperTriangles(
        d, vtkm_anari::ANARIActor(tangleIso), "isosurface"));
    setTF(d, mIso);
  } else {
    auto &mIso = scene.AddMapper(vtkm_anari::ANARIMapperPoints(
        d, vtkm_anari::ANARIActor(tangleIso), "isosurface"));
    setTF(d, mIso);
  }

  // Render a frame //

  auto renderer = anari::newObject<anari::Renderer>(d, "raycast");
  anari::setParameter(
      d, renderer, "backgroundColor", glm::vec4(0.f, 0.f, 0.f, 1.f));
  anari::commitParameters(d, renderer);

  auto camera = anari::newObject<anari::Camera>(d, "perspective");
  anari::setParameter(d, camera, "aspect", 1024 / float(768));
  anari::setParameter(d, camera, "position", glm::vec3(-0.05, 1.43, 1.87));
  anari::setParameter(d, camera, "direction", glm::vec3(0.32, -0.53, -0.79));
  anari::setParameter(d, camera, "up", glm::vec3(-0.20, -0.85, 0.49));
  anari::commitParameters(d, camera);

  auto frame = anari::newObject<anari::Frame>(d);
  anari::setParameter(d, frame, "size", glm::uvec2(1024, 768));
  anari::setParameter(d, frame, "color", ANARI_UFIXED8_RGBA_SRGB);
  anari::setParameter(d, frame, "world", scene.GetANARIWorld());
  anari::setParameter(d, frame, "camera", camera);
  anari::setParameter(d, frame, "renderer", renderer);
  anari::commitParameters(d, frame);

  anari::release(d, camera);
  anari::release(d, renderer);

  for (auto _ : state) {
    anari::render(d, frame);
    anari::wait(d, frame);
  }

  const auto fb = anari::map<uint32_t>(d, frame, "color");
  std::string filename = "benchmark_isosurface_"
      + (TRIANGLES ? std::string("triangles_") : std::string("points"))
      + std::to_string(size) + ".png";
  stbi_write_png(filename.c_str(),
      int(fb.width),
      int(fb.height),
      4,
      fb.data,
      4 * int(fb.width));
  anari::unmap(d, frame, "color");

  // Cleanup //

  anari::release(d, frame);
}

static void bench_isosurface_triangles(benchmark::State &state)
{
  bench_isosurface<true>(state);
}

static void bench_isosurface_points(benchmark::State &state)
{
  bench_isosurface<false>(state);
}

BENCHMARK(bench_isosurface_triangles)
    ->RangeMultiplier(4)
    ->Range(8, 512)
    ->Unit(benchmark::kMillisecond);
BENCHMARK(bench_isosurface_points)
    ->RangeMultiplier(4)
    ->Range(8, 512)
    ->Unit(benchmark::kMillisecond);

static void bench_isosurface_value_sweep(benchmark::State &state)
{
  // Create VTKm datasets //

  auto tangle = vtkm::source::Tangle(vtkm::Id3{128}).Execute();

  auto getNormalizedIsovalue = [](auto v) { return v % 100 / 100.f; };
  size_t isovalue = 0;
  auto tangleIso = makeIsosurface(tangle, getNormalizedIsovalue(isovalue++));

  // Map data to ANARI objects //

  vtkm_anari::ANARIScene scene(d);

  auto &mIso = scene.AddMapper(vtkm_anari::ANARIMapperTriangles(
      d, vtkm_anari::ANARIActor(tangleIso), "isosurface"));
  setTF(d, mIso);

  // Render a frame //

  auto renderer = anari::newObject<anari::Renderer>(d, "raycast");
  anari::setParameter(
      d, renderer, "backgroundColor", glm::vec4(0.f, 0.f, 0.f, 1.f));
  anari::commitParameters(d, renderer);

  auto camera = anari::newObject<anari::Camera>(d, "perspective");
  anari::setParameter(d, camera, "aspect", 1024 / float(768));
  anari::setParameter(d, camera, "position", glm::vec3(-0.05, 1.43, 1.87));
  anari::setParameter(d, camera, "direction", glm::vec3(0.32, -0.53, -0.79));
  anari::setParameter(d, camera, "up", glm::vec3(-0.20, -0.85, 0.49));
  anari::commitParameters(d, camera);

  auto frame = anari::newObject<anari::Frame>(d);
  anari::setParameter(d, frame, "size", glm::uvec2(1024, 768));
  anari::setParameter(d, frame, "color", ANARI_UFIXED8_RGBA_SRGB);
  anari::setParameter(d, frame, "world", scene.GetANARIWorld());
  anari::setParameter(d, frame, "camera", camera);
  anari::setParameter(d, frame, "renderer", renderer);
  anari::commitParameters(d, frame);

  anari::release(d, camera);
  anari::release(d, renderer);

  for (auto _ : state) {
    state.PauseTiming();
    tangleIso = makeIsosurface(tangle, getNormalizedIsovalue(isovalue++));
    mIso.SetActor(vtkm_anari::ANARIActor(tangleIso));
    state.ResumeTiming();
    anari::render(d, frame);
    anari::wait(d, frame);
  }

  const auto fb = anari::map<uint32_t>(d, frame, "color");
  std::string filename = "benchmark_isosurface_value_sweep.png";
  stbi_write_png(filename.c_str(),
      int(fb.width),
      int(fb.height),
      4,
      fb.data,
      4 * int(fb.width));
  anari::unmap(d, frame, "color");

  // Cleanup //

  anari::release(d, frame);
}

BENCHMARK(bench_isosurface_value_sweep)->Unit(benchmark::kMillisecond);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  lib = anari::loadLibrary("visrtx", anariStatusFunc);
  d = anari::newDevice(lib, "default");

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();

  anari::release(d, d);
  anari::unloadLibrary(lib);
}
