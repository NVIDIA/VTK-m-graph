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

namespace vtkm_anari {
namespace graph {

struct VTKM_ANARI_EXPORT ExecutionGraph : public NodeObserver
{
  ExecutionGraph(anari::Device d);
  ~ExecutionGraph();

  // Not copyable or movable

  ExecutionGraph(const ExecutionGraph &) = delete;
  ExecutionGraph(ExecutionGraph &&) = delete;
  ExecutionGraph &operator=(const ExecutionGraph &) = delete;
  ExecutionGraph &operator=(ExecutionGraph &&) = delete;

  // Add/remove nodes //

  template <typename T, typename... Args>
  T *addNode(Args &&...args);
  void removeNode(int id);

  // Node iteration //

  size_t getNumberOfNodes() const;
  Node *getNode(size_t i) const;

  // Scene //

  anari::World getANARIWorld() const;

  void updateWorld();

  int numVisibleMappers() const;

  const TimeStamp &lastChange() const;

  // Utility //

  void print();

 private:
  void nodeChanged(Node *) override;

  std::vector<NodePtr> m_nodes;
  std::vector<MapperNode *> m_mapperNodes;

  TimeStamp m_lastChange;
  bool m_needToUpdate{true};

  int m_numVisibleMappers{0};

  mutable ANARIScene m_scene;
};

///////////////////////////////////////////////////////////////////////////////
// Inlined definitions ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T, typename... Args>
inline T *ExecutionGraph::addNode(Args &&...args)
{
  static_assert(std::is_base_of<Node, T>::value,
      "ExecutionGraph::addNode() can only construct types derived from Node.");
  auto *node = new T(std::forward<Args>(args)...);
  m_nodes.emplace_back(node);
  node->setObserver(this);
  if (node->type() == NodeType::MAPPER) {
    auto *mn = (MapperNode *)node;
    m_mapperNodes.push_back(mn);
    mn->addMapperToScene(m_scene, {});
  }
  return node;
}

} // namespace graph
} // namespace vtkm_anari
