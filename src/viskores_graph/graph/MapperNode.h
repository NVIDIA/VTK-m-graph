// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Node.h"
// viskores
#include <viskores/interop/anari/ANARIScene.h>

namespace viskores {
namespace graph {

struct VISKORES_GRAPH_EXPORT MapperNode : public Node
{
  MapperNode();
  ~MapperNode() override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  NodeType type() const override;
  bool isValid() const override;

  void parameterChanged(Parameter *p, ParameterChangeType type) override;

  bool isVisible() const;
  bool isMapperEmpty() const;

  interop::anari::ANARIMapper *getMapper() const;

  void update() override;
  void updateUpstreamNodes();

  virtual void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) = 0;

 protected:
  interop::anari::ANARIMapper *m_mapper{nullptr};
  interop::anari::ANARIScene *m_scene{nullptr};

 private:
  bool m_visible{true};
  InPort m_actorPort{PortType::ACTOR, "actor", this};
};

// Concrete node types ////////////////////////////////////////////////////////

struct VISKORES_GRAPH_EXPORT GlyphMapperNode : public MapperNode
{
  GlyphMapperNode() = default;
  const char *kind() const override;

 private:
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

struct VISKORES_GRAPH_EXPORT PointMapperNode : public MapperNode
{
  PointMapperNode() = default;
  const char *kind() const override;

 private:
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

struct VISKORES_GRAPH_EXPORT TriangleMapperNode : public MapperNode
{
  TriangleMapperNode();
  const char *kind() const override;

 private:
  void parameterChanged(Parameter *p, ParameterChangeType type) override;
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

struct VISKORES_GRAPH_EXPORT VolumeMapperNode : public MapperNode
{
  VolumeMapperNode() = default;
  const char *kind() const override;

 private:
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

} // namespace graph
} // namespace viskores
