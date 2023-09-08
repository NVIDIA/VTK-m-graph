/*
 * Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <vtkm/rendering/Actor.h>
#include <vtkm/rendering/CanvasRayTracer.h>
#include <vtkm/rendering/MapperPoint.h>
#include <vtkm/rendering/MapperRayTracer.h>
#include <vtkm/rendering/MapperWireframer.h>
#include <vtkm/rendering/Scene.h>
#include <vtkm/rendering/View3D.h>
#include <vtkm/source/Tangle.h>
// stb
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
// anari
#include <anari/anari_cpp.hpp>

// ANARI C++ specializations //////////////////////////////////////////////////

namespace anari {
ANARI_TYPEFOR_SPECIALIZATION(vtkm::Vec2f_32, ANARI_FLOAT32_VEC2);
ANARI_TYPEFOR_SPECIALIZATION(vtkm::Vec3f_32, ANARI_FLOAT32_VEC3);
ANARI_TYPEFOR_SPECIALIZATION(vtkm::Vec4f_32, ANARI_FLOAT32_VEC4);
ANARI_TYPEFOR_DEFINITION(vtkm::Vec2f_32);
ANARI_TYPEFOR_DEFINITION(vtkm::Vec3f_32);
ANARI_TYPEFOR_DEFINITION(vtkm::Vec4f_32);
} // namespace anari

// Global data/config /////////////////////////////////////////////////////////

constexpr uint32_t frameSize[] = {1024, 768};

anari::Library lib = nullptr;
anari::Device d = nullptr;

vtkm::rendering::CanvasRayTracer canvas(frameSize[0], frameSize[1]);
vtkm::cont::ColorTable colorTable; // use default
vtkm::rendering::Camera camera;

const vtkm::Vec3f_32 position(1.609451, 1.519826, -0.590263);
const vtkm::Vec3f_32 lookat(0.500000, 0.500000, 0.500000);
const vtkm::Vec3f_32 up(-0.391069, 0.836286, 0.384305);
const vtkm::Vec4f_32 backgroundColor(0.2f, 0.2f, 0.2f, 1.f);

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
    fprintf(stderr, "[ANARI][FATAL]: %s\n", message);
    std::exit(1);
  } else if (severity == ANARI_SEVERITY_ERROR) {
    fprintf(stderr, "[ANARI][ERROR]: %s\n", message);
  }
}

static void setTF(anari::Device d, vtkm_anari::ANARIMapper &mapper)
{
  constexpr int numSamples = 128;
  vtkm::cont::ColorTableSamplesRGBA samples;
  samples.NumberOfSamples = numSamples;

  auto colorArray = anari::newArray1D(d, ANARI_FLOAT32_VEC3, numSamples);
  auto *colors = anari::map<vtkm::Vec3f_32>(d, colorArray);

  if (colorTable.Sample(numSamples, samples)) {
    vtkm::cont::Token token;
    auto *p =
        (const vtkm::Vec4ui_8 *)samples.Samples.GetBuffers()[0].ReadPointerHost(
            token);
    for (int i = 0; i < numSamples; i++)
      colors[i] = vtkm::Vec3f_32(p[i][0], p[i][1], p[i][2]) / 255.f;
  } else {
    std::fill(colors, colors + numSamples, vtkm::Vec3f_32(0.f));
  }

  anari::unmap(d, colorArray);

  auto opacityArray = anari::newArray1D(d, ANARI_FLOAT32, 2);
  auto *opacities = anari::map<float>(d, opacityArray);

  opacities[0] = 0.f;
  opacities[1] = 1.f;

  anari::unmap(d, opacityArray);

  mapper.SetANARIColorMapArrays(
      colorArray, nullptr, opacityArray, nullptr, true);
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

static anari::Frame makeANARIFrame(anari::Device d, anari::World w)
{
  auto renderer = anari::newObject<anari::Renderer>(d, "raycast");
  anari::setParameter(d, renderer, "backgroundColor", backgroundColor);
  anari::commitParameters(d, renderer);

  auto camera = anari::newObject<anari::Camera>(d, "perspective");
  anari::setParameter(d, camera, "aspect", frameSize[0] / float(frameSize[1]));
  anari::setParameter(d, camera, "position", position);
  anari::setParameter(d, camera, "direction", lookat - position);
  anari::setParameter(d, camera, "up", -up);
  anari::commitParameters(d, camera);

  auto frame = anari::newObject<anari::Frame>(d);
  anari::setParameter(d, frame, "size", frameSize);
  anari::setParameter(d, frame, "color", ANARI_UINT8_VEC4);
  anari::setParameter(d, frame, "world", w);
  anari::setParameter(d, frame, "camera", camera);
  anari::setParameter(d, frame, "renderer", renderer);
  anari::commitParameters(d, frame);

  anari::release(d, camera);
  anari::release(d, renderer);

  return frame;
}

///////////////////////////////////////////////////////////////////////////////
// Benchmarks /////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static void vtkm_isosurface_value_sweep(benchmark::State &state)
{
  // Create VTKm datasets //

  auto tangle = vtkm::source::Tangle(vtkm::Id3{128}).Execute();
  vtkm::Range range;
  tangle.GetField(0).GetRange(&range);

  auto getNormalizedIsovalue = [](auto v) { return v % 100 / 100.f; };
  size_t isovalue = 0;
  auto tangleIso = makeIsosurface(tangle, getNormalizedIsovalue(isovalue++));

  // Render frames //

  for (auto _ : state) {
    state.PauseTiming();
    tangleIso = makeIsosurface(tangle, getNormalizedIsovalue(isovalue++));
    state.ResumeTiming();
    vtkm::rendering::Scene scene;
    vtkm::rendering::Actor actor(tangleIso.GetCellSet(),
        tangleIso.GetCoordinateSystem(),
        tangleIso.GetField(0),
        colorTable);
    actor.SetScalarRange(range);
    scene.AddActor(actor);
    vtkm::rendering::View3D view(scene,
        vtkm::rendering::MapperRayTracer(),
        canvas,
        camera,
        backgroundColor);
    view.SetWorldAnnotationsEnabled(false);
    view.SetRenderAnnotationsEnabled(false);
    view.Paint();
  }

  tangleIso = makeIsosurface(tangle, 0.5f);
  vtkm::rendering::Actor actor(tangleIso.GetCellSet(),
      tangleIso.GetCoordinateSystem(),
      tangleIso.GetField(0),
      colorTable);
  actor.SetScalarRange(range);
  vtkm::rendering::Scene scene;
  scene.AddActor(actor);
  vtkm::rendering::View3D view(scene,
      vtkm::rendering::MapperRayTracer(),
      canvas,
      camera,
      backgroundColor);
  view.SetWorldAnnotationsEnabled(false);
  view.SetRenderAnnotationsEnabled(false);
  view.Paint();
  view.SaveAs("vtkm_isosurface_value_sweep.png");
}

BENCHMARK(vtkm_isosurface_value_sweep)->Unit(benchmark::kMillisecond);

template <bool TRIANGLES>
static void bench_vtkm_isosurface(benchmark::State &state)
{
  // Create VTKm datasets //

  const auto size = state.range(0);
  auto tangle = vtkm::source::Tangle(vtkm::Id3{size}).Execute();
  auto tangleIso = makeIsosurface(tangle, 0.25f);

  // Map data to renderable objects //

  vtkm::Range range;
  tangle.GetField(0).GetRange(&range);
  vtkm::rendering::Actor actor(tangleIso.GetCellSet(),
      tangleIso.GetCoordinateSystem(),
      tangleIso.GetField(0),
      colorTable);
  actor.SetScalarRange(range);

  vtkm::rendering::Scene scene;
  scene.AddActor(actor);

  std::unique_ptr<vtkm::rendering::View3D> view;
  if (TRIANGLES) {
    view = std::make_unique<vtkm::rendering::View3D>(scene,
        vtkm::rendering::MapperRayTracer(),
        canvas,
        camera,
        backgroundColor);
  } else {
    view = std::make_unique<vtkm::rendering::View3D>(
        scene, vtkm::rendering::MapperPoint(), canvas, camera, backgroundColor);
  }
  view->SetWorldAnnotationsEnabled(false);
  view->SetRenderAnnotationsEnabled(false);

  // Render frames //

  for (auto _ : state)
    view->Paint();

  std::string filename = "vtkm_isosurface_"
      + (TRIANGLES ? std::string("triangles_") : std::string("points_"))
      + std::to_string(size) + ".png";
  view->SaveAs(filename);
}

static void vtkm_isosurface_triangles(benchmark::State &state)
{
  bench_vtkm_isosurface<true>(state);
}

static void vtkm_isosurface_points(benchmark::State &state)
{
  bench_vtkm_isosurface<false>(state);
}

BENCHMARK(vtkm_isosurface_triangles)
    ->RangeMultiplier(4)
    ->Range(8, 512)
    ->Unit(benchmark::kMillisecond);
BENCHMARK(vtkm_isosurface_points)
    ->RangeMultiplier(4)
    ->Range(8, 512)
    ->Unit(benchmark::kMillisecond);

template <bool TRIANGLES>
static void bench_anari_isosurface(benchmark::State &state)
{
  // Create VTKm datasets //

  const auto size = state.range(0);
  auto tangle = vtkm::source::Tangle(vtkm::Id3{size}).Execute();
  auto tangleIso = makeIsosurface(tangle, 0.25f);

  // Map data to ANARI objects //

  vtkm::Range range;
  tangle.GetField(0).GetRange(&range);

  vtkm_anari::ANARIScene scene(d);

  if (TRIANGLES) {
    auto &mIso = scene.AddMapper(vtkm_anari::ANARIMapperTriangles(
        d, vtkm_anari::ANARIActor(tangleIso), "isosurface"));
    setTF(d, mIso);
    mIso.SetANARIColorMapValueRange(vtkm::Vec2f_32(range.Min, range.Max));
  } else {
    auto &mIso = scene.AddMapper(vtkm_anari::ANARIMapperPoints(
        d, vtkm_anari::ANARIActor(tangleIso), "isosurface"));
    setTF(d, mIso);
    mIso.SetANARIColorMapValueRange(vtkm::Vec2f_32(range.Min, range.Max));
  }

  // Render frames //

  auto frame = makeANARIFrame(d, scene.GetANARIWorld());

  for (auto _ : state) {
    anari::render(d, frame);
    anari::wait(d, frame);
  }

  const auto fb = anari::map<uint32_t>(d, frame, "color");
  std::string filename = "anari_isosurface_"
      + (TRIANGLES ? std::string("triangles_") : std::string("points_"))
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

static void anari_isosurface_triangles(benchmark::State &state)
{
  bench_anari_isosurface<true>(state);
}

static void anari_isosurface_points(benchmark::State &state)
{
  bench_anari_isosurface<false>(state);
}

BENCHMARK(anari_isosurface_triangles)
    ->RangeMultiplier(4)
    ->Range(8, 512)
    ->Unit(benchmark::kMillisecond);
BENCHMARK(anari_isosurface_points)
    ->RangeMultiplier(4)
    ->Range(8, 512)
    ->Unit(benchmark::kMillisecond);

static void anari_isosurface_value_sweep(benchmark::State &state)
{
  // Create VTKm datasets //

  auto tangle = vtkm::source::Tangle(vtkm::Id3{128}).Execute();

  auto getNormalizedIsovalue = [](auto v) { return v % 100 / 100.f; };
  size_t isovalue = 0;
  auto tangleIso = makeIsosurface(tangle, getNormalizedIsovalue(isovalue++));

  // Map data to ANARI objects //

  vtkm::Range range;
  tangle.GetField(0).GetRange(&range);

  vtkm_anari::ANARIScene scene(d);

  auto &mIso = scene.AddMapper(vtkm_anari::ANARIMapperTriangles(
      d, vtkm_anari::ANARIActor(tangleIso), "isosurface"));
  setTF(d, mIso);
  mIso.SetANARIColorMapValueRange(vtkm::Vec2f_32(range.Min, range.Max));

  // Render frames //

  auto frame = makeANARIFrame(d, scene.GetANARIWorld());

  for (auto _ : state) {
    state.PauseTiming();
    tangleIso = makeIsosurface(tangle, getNormalizedIsovalue(isovalue++));
    mIso.SetActor(vtkm_anari::ANARIActor(tangleIso));
    setTF(d, mIso);
    mIso.SetANARIColorMapValueRange(vtkm::Vec2f_32(range.Min, range.Max));
    state.ResumeTiming();
    anari::render(d, frame);
    anari::wait(d, frame);
  }

  tangleIso = makeIsosurface(tangle, 0.5f);
  mIso.SetActor(vtkm_anari::ANARIActor(tangleIso));
  setTF(d, mIso);
  mIso.SetANARIColorMapValueRange(vtkm::Vec2f_32(range.Min, range.Max));
  anari::render(d, frame);
  anari::wait(d, frame);

  const auto fb = anari::map<uint32_t>(d, frame, "color");
  std::string filename = "anari_isosurface_value_sweep.png";
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

BENCHMARK(anari_isosurface_value_sweep)->Unit(benchmark::kMillisecond);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  lib = anari::loadLibrary("visrtx", anariStatusFunc);
  d = anari::newDevice(lib, "default");

  camera.SetLookAt(lookat);
  camera.SetViewUp(up);
  camera.SetPosition(position);
  camera.SetClippingRange(0.1f, 1e30f);
  camera.SetFieldOfView(60.f);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();

  anari::release(d, d);
  anari::unloadLibrary(lib);
}
