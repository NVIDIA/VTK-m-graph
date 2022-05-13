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

#include "graph/ActorNode.h"
#include "graph/FilterNode.h"
#include "graph/MapperNode.h"
#include "graph/SourceNode.h"

namespace vtkm_anari::graph {

struct VTKM_ANARI_EXPORT ExecutionGraph : public NodeObserver
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

  template <typename T, typename... Args>
  FilterNode *addFilterNode(Args &&...args);

  ActorNode *addActorNode();

  template <typename T, typename... Args>
  MapperNode *addMapperNode(Args &&...args);

  void removeSourceNode(int id);
  void removeFilterNode(int id);
  void removeActorNode(int id);
  void removeMapperNode(int id);

  void removeNode(int id); // try all lists because type is unknown

  // Node iteration //

  size_t getNumberOfNodes() const;

  size_t getNumberOfSourceNodes() const;
  size_t getNumberOfFilterNodes() const;
  size_t getNumberOfActorNodes() const;
  size_t getNumberOfMapperNodes() const;

  SourceNode *getSourceNode(size_t i) const;
  FilterNode *getFilterNode(size_t i) const;
  ActorNode *getActorNode(size_t i) const;
  MapperNode *getMapperNode(size_t i) const;

  // Scene //

  anari::World getANARIWorld() const;

  void updateWorld();

  // Utility //

  void print();

 private:
  void nodeChanged(Node *) override;

  std::vector<SourceNodePtr> m_sourceNodes;
  std::vector<FilterNodePtr> m_filterNodes;
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
  auto *node = m_sourceNodes.back().get();
  node->setObserver(this);
  return node;
}

template <typename T, typename... Args>
inline FilterNode *ExecutionGraph::addFilterNode(Args &&...args)
{
  static_assert(std::is_base_of<FilterNode, T>::value,
      "ExecutionGraph::addFilterNode() can only construct types derived"
      "from FilterNode.");
  m_filterNodes.emplace_back(new T(std::forward<Args>(args)...));
  auto *node = m_filterNodes.back().get();
  node->setObserver(this);
  return node;
}

template <typename T, typename... Args>
inline MapperNode *ExecutionGraph::addMapperNode(Args &&...args)
{
  static_assert(std::is_base_of<MapperNode, T>::value,
      "ExecutionGraph::addMapperNode() can only construct types derived"
      "from MapperNode.");
  m_mapperNodes.emplace_back(new T(std::forward<Args>(args)...));
  auto *node = m_mapperNodes.back().get();
  node->setObserver(this);
  return node;
}

} // namespace vtkm_anari::graph