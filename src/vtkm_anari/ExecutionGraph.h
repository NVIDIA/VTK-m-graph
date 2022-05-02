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

// std
#include <memory>
#include <string>
#include <vector>
// vtk-m
#include <vtkm/cont/DataSet.h>

#include "ANARIMapper.h"
#include "ANARIScene.h"

namespace vtkm_anari::graph {

// Port ///////////////////////////////////////////////////////////////////////

enum class PortType
{
  DATASET,
  ACTOR,
  UNKNOWN
};

struct Node;

struct Port
{
  Port(PortType type, std::string name, Node *node);
  virtual ~Port() = default;

  PortType type() const;
  const char *name() const;
  Node *node();

  // Not copyable or movable
  Port(const Port &) = delete;
  Port(Port &&) = delete;
  Port &operator=(const Port &) = delete;
  Port &operator=(Port &&) = delete;

 private:
  PortType m_type{PortType::UNKNOWN};
  std::string m_name;
  Node *m_node{nullptr};
};

struct OutPort;

struct InPort : public Port
{
  InPort(PortType type, std::string name, Node *node);
  ~InPort() override;

  int id() const;

  bool isConnected() const;

  bool connect(OutPort *from);
  void disconnect();
  OutPort *other() const;

  static InPort *fromID(int id);

 private:
  OutPort *m_connection{nullptr};
  int m_id{-1};
};

struct OutPort : public Port
{
  OutPort(PortType type, std::string name, Node *node);
  ~OutPort() override;

  int id() const;

  bool connect(InPort *from);
  void disconnect(InPort *from);

  static OutPort *fromID(int id);

 private:
  std::vector<InPort *> m_connections;
  int m_id{-1};
};

bool connect(OutPort *from, InPort *to);

// Node ///////////////////////////////////////////////////////////////////////

struct Node
{
  Node();
  ~Node();

  virtual bool isValid() const = 0;

  const char *name() const;
  virtual const char *kind() const = 0;
  int id() const;

  virtual InPort *input(const char *name);
  virtual OutPort *output(const char *name);

 private:
  mutable std::string m_name;
  int m_id{-1};
};

struct SourceNode : public Node
{
  SourceNode() = default;

  virtual vtkm::cont::DataSet dataset() = 0;
  OutPort *output(const char *name) override;

  bool isValid() const override;

 private:
  OutPort m_datasetPort{PortType::DATASET, "dataset", this};
};

struct TangleSourceNode : public SourceNode
{
  TangleSourceNode() = default;
  const char *kind() const override;
  vtkm::cont::DataSet dataset() override;
};

struct ActorNode : public Node
{
  ActorNode() = default;
  const char *kind() const override;

  InPort *input(const char *name) override;
  OutPort *output(const char *name) override;

  bool isValid() const override;

  ANARIActor makeActor(vtkm::cont::DataSet ds);

 private:
  InPort m_datasetPort{PortType::DATASET, "dataset", this};
  OutPort m_actorPort{PortType::ACTOR, "actor", this};
};

struct MapperNode : public Node
{
  MapperNode() = default;

  InPort *input(const char *name) override;

  bool isValid() const override;

  virtual void addMapperToScene(ANARIScene &scene, ANARIActor a) = 0;

 private:
  InPort m_actorPort{PortType::ACTOR, "actor", this};
};

struct VolumeMapperNode : public MapperNode
{
  VolumeMapperNode() = default;
  const char *kind() const override;
  void addMapperToScene(ANARIScene &scene, ANARIActor a) override;
};

using SourceNodePtr = std::unique_ptr<SourceNode>;
using ActorNodePtr = std::unique_ptr<ActorNode>;
using MapperNodePtr = std::unique_ptr<MapperNode>;

// Graph //////////////////////////////////////////////////////////////////////

struct ExecutionGraph
{
  ExecutionGraph(anari::Device d);

  // Not copyable or movable

  ExecutionGraph(const ExecutionGraph &) = delete;
  ExecutionGraph(ExecutionGraph &&) = delete;
  ExecutionGraph &operator=(const ExecutionGraph &) = delete;
  ExecutionGraph &operator=(ExecutionGraph &&) = delete;

  // Add/remove nodes //

  template <typename T, typename... Args>
  SourceNode *addSourceNode(Args &&...args);

  ActorNode *addActorNode();

  template <typename T, typename... Args>
  MapperNode *addMapperNode(Args &&...args);

  void removeSourceNode(int id);
  void removeActorNode(int id);
  void removeMapperNode(int id);

  // Node iteration //

  size_t getNumberOfSourceNodes() const;
  size_t getNumberOfActorNodes() const;
  size_t getNumberOfMapperNodes() const;

  SourceNode *getSourceNode(size_t i) const;
  ActorNode *getActorNode(size_t i) const;
  MapperNode *getMapperNode(size_t i) const;

  // Scene //

  anari::World getANARIWorld() const;

  void updateWorld();

  // Utility //

  void print();

 private:
  std::vector<SourceNodePtr> m_sourceNodes;
  std::vector<ActorNodePtr> m_actorNodes;
  std::vector<MapperNodePtr> m_mapperNodes;

  mutable ANARIScene m_scene;
};

///////////////////////////////////////////////////////////////////////////////
// Inlined definitions ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T, typename... Args>
inline SourceNode *ExecutionGraph::addSourceNode(Args &&...args)
{
  static_assert(std::is_base_of<SourceNode, T>::value,
      "ExecutionGraph::addSourceNode() can only construct types derived"
      "from SourceNode.");
  m_sourceNodes.emplace_back(new T(std::forward<Args>(args)...));
  return m_sourceNodes.back().get();
}

template <typename T, typename... Args>
inline MapperNode *ExecutionGraph::addMapperNode(Args &&...args)
{
  static_assert(std::is_base_of<MapperNode, T>::value,
      "ExecutionGraph::addMapperNode() can only construct types derived"
      "from MapperNode.");
  m_mapperNodes.emplace_back(new T(std::forward<Args>(args)...));
  return m_mapperNodes.back().get();
}

} // namespace vtkm_anari::graph