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
#include "vtkm_anari/ANARIMapperPoints.h"
#include "vtkm_anari/ANARIMapperTriangles.h"
#include "vtkm_anari/ANARIMapperVolume.h"
#include "vtkm_anari/ANARIScene.h"
// vtk-m
#include <vtkm/filter/Contour.h>
#include <vtkm/filter/Gradient.h>
#include <vtkm/source/Tangle.h>
// stb
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
// anari
#include <anari/anari_cpp/ext/glm.h>
// std
#include <vector>

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

int main()
{
  stbi_flip_vertically_on_write(1);

  {
    // Initialize ANARI ///////////////////////////////////////////////////////

    printf("initialize ANARI...");

#define USE_DEBUG_DEVICE 0

    auto debug_lib = anari::loadLibrary("debug", anariStatusFunc);

#if 1
    auto lib = anari::loadLibrary("visrtx", anariStatusFunc);
#else
    auto lib = anari::loadLibrary("sink", anariStatusFunc);
#endif

#if USE_DEBUG_DEVICE
    auto nested = anari::newDevice(lib, "default");
    auto d = anari::newDevice(debug_lib, "debug");
    anari::setParameter(d, d, "wrappedDevice", ANARI_DEVICE, &nested);
    anari::commitParameters(d, d);
#else
    auto d = anari::newDevice(lib, "default");
#endif

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

    vtkm::filter::Contour contourFilter;
    contourFilter.SetIsoValue(isovalue);
    contourFilter.SetActiveField(tangle_field.GetName());
    auto tangleIso = contourFilter.Execute(tangle);

    vtkm::filter::Gradient gradientFilter;
    gradientFilter.SetActiveField(tangle_field.GetName());
    gradientFilter.SetOutputFieldName("Gradient");
    auto tangleGrad = gradientFilter.Execute(tangle);

    printf("done\n");

    // Map data to ANARI objects //////////////////////////////////////////////

    printf("mapping VTKm datasets to anari::World...");

    vtkm_anari::ANARIScene scene(d);

    {
#if 0
      vtkm_anari::ANARIActor va(
          tangle.GetCellSet(), tangle.GetCoordinateSystem(), tangle_field);
      vtkm_anari::ANARIMapperVolume mVol(d, va, "volume");
      setTF(d, mVol);

      vtkm_anari::ANARIActor sa(tangleIso.GetCellSet(),
          tangleIso.GetCoordinateSystem(),
#if 1 // include field data
          tangleIso.GetField(0));
#else // empty field
          {});
#endif

#if 1 // isosurface as triangle mesh
      vtkm_anari::ANARIMapperTriangles mIso(d, sa, "isosurface");
      mIso.SetCalculateNormals(true);
#else // isosurface as points
      vtkm_anari::ANARIMapperPoints mIso(d, sa, "isosurface");
#endif

#if 0 // set the field as an ANARI geometry attribute? (ignored if empty field)
      mIso.SetMapFieldAsAttribute(false);
#endif
      setTF(d, mIso);

      vtkm_anari::ANARIActor ga(tangleGrad.GetCellSet(),
          tangleGrad.GetCoordinateSystem(),
          tangleGrad.GetField(0));
#if 0 // test out adding a mapper with the same name
      vtkm_anari::ANARIMapperGlyphs mGrad(d, ga, "isosurface");
#else // give it a unique name (normal behavior)
      vtkm_anari::ANARIMapperGlyphs mGrad(d, ga, "gradient");
#endif
      setTF(d, mGrad);

      scene.AddMapper(mVol);
      scene.AddMapper(mIso);
      scene.AddMapper(mGrad);
#else
      auto &mVol = scene.AddMapper(vtkm_anari::ANARIMapperVolume(d,
          vtkm_anari::ANARIActor(
              tangle.GetCellSet(), tangle.GetCoordinateSystem(), tangle_field),
          "volume"));
      auto &mIso = scene.AddMapper(vtkm_anari::ANARIMapperTriangles(d,
          vtkm_anari::ANARIActor(
              tangleIso.GetCellSet(), tangleIso.GetCoordinateSystem(), {}),
          "isosurface"));
      auto &mGrad = scene.AddMapper(vtkm_anari::ANARIMapperGlyphs(d,
          vtkm_anari::ANARIActor(tangleGrad.GetCellSet(),
              tangleGrad.GetCoordinateSystem(),
              tangleGrad.GetField(0)),
          "gradient"));

      setTF(d, mVol);
      setTF(d, mIso);
      setTF(d, mGrad);

      mIso.SetCalculateNormals(true);
#endif

#if 0 // hide gradient glyphs (unset this if using the name "isosurface")
      scene.SetMapperVisible(2, false);
#endif
    }

    printf("done\n");

    printf("mappers added to scene: {'%s'", scene.GetMapper(0).GetName());
    for (size_t i = 1; i < scene.GetNumberOfMappers(); i++)
      printf(",'%s'", scene.GetMapper(i).GetName());
    printf("}\n");

    if (scene.HasMapperWithName("isosurface"))
      printf("scene has mapper 'isosurface'\n");

#if 0 // try out removing a mapper
    scene.RemoveMapper("gradient");
#endif

    // Render a frame /////////////////////////////////////////////////////////

    printf("creating anari::Frame and rendering it...");

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

    anari::render(d, frame);
    anari::wait(d, frame);

    const auto fb = anari::map<uint32_t>(d, frame, "color");
    stbi_write_png("scene.png",
        int(fb.width),
        int(fb.height),
        4,
        fb.data,
        4 * int(fb.width));
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