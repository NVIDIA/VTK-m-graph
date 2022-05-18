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

#pragma once

#include "../ANARIScene.h"
#include "Node.h"

namespace vtkm_anari {
namespace graph {

struct VTKM_ANARI_EXPORT MapperNode : public Node
{
  MapperNode() = default;
  ~MapperNode() override;

  InPort *input(const char *name) override;

  NodeType type() const override;
  bool isValid() const override;

  bool isVisible() const;
  void setVisible(bool show);

  virtual void addMapperToScene(ANARIScene &scene, ANARIActor a) = 0;

 private:
  bool m_visible{true};
  InPort m_actorPort{PortType::ACTOR, "actor", this};
};

using MapperNodePtr = std::unique_ptr<MapperNode>;

// Concrete node types ////////////////////////////////////////////////////////

struct VTKM_ANARI_EXPORT VolumeMapperNode : public MapperNode
{
  VolumeMapperNode() = default;
  const char *kind() const override;
  void addMapperToScene(ANARIScene &scene, ANARIActor a) override;
};

struct VTKM_ANARI_EXPORT TriangleMapperNode : public MapperNode
{
  TriangleMapperNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;
  void addMapperToScene(ANARIScene &scene, ANARIActor a) override;

 private:
  ANARIMapper *m_mapper{nullptr};
};

struct VTKM_ANARI_EXPORT PointMapperNode : public MapperNode
{
  PointMapperNode() = default;
  const char *kind() const override;
  void addMapperToScene(ANARIScene &scene, ANARIActor a) override;
};

struct VTKM_ANARI_EXPORT GlyphMapperNode : public MapperNode
{
  GlyphMapperNode() = default;
  const char *kind() const override;
  void addMapperToScene(ANARIScene &scene, ANARIActor a) override;
};

} // namespace graph
} // namespace vtkm_anari
