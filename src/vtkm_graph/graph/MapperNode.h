// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Node.h"
// vtk-m
#include <vtkm/interop/anari/ANARIScene.h>

namespace vtkm {
namespace graph {

struct VTKM_GRAPH_EXPORT MapperNode : public Node
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

struct VTKM_GRAPH_EXPORT GlyphMapperNode : public MapperNode
{
  GlyphMapperNode() = default;
  const char *kind() const override;

 private:
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

struct VTKM_GRAPH_EXPORT PointMapperNode : public MapperNode
{
  PointMapperNode() = default;
  const char *kind() const override;

 private:
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

struct VTKM_GRAPH_EXPORT TriangleMapperNode : public MapperNode
{
  TriangleMapperNode();
  const char *kind() const override;

 private:
  void parameterChanged(Parameter *p, ParameterChangeType type) override;
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

struct VTKM_GRAPH_EXPORT VolumeMapperNode : public MapperNode
{
  VolumeMapperNode() = default;
  const char *kind() const override;

 private:
  void addMapperToScene(
      interop::anari::ANARIScene &scene, interop::anari::ANARIActor a) override;
};

} // namespace graph
} // namespace vtkm
