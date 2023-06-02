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
#include "graph/MapperNode.h"
#include "graph/FilterNode.h"
#include "graph/SourceNode.h"
#include "graph/UtilityNode.h"
// std
#include <functional>
#include <future>

namespace vtkm {
namespace graph {

using GraphUpdateCallback = std::function<void()>;

struct DeferredParameterUpdateValue
{
  Parameter *p{nullptr};
  ParameterRawValue v;
};

struct VTKM_GRAPH_EXPORT ExecutionGraph : public NodeObserver
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
  T *addNamedNode(const std::string &name, Args &&...args);
  template <typename T, typename... Args>
  T *addNode(Args &&...args);
  void removeNode(int id);

  // Node iteration //

  size_t getNumberOfNodes() const;
  Node *getNode(size_t i) const;

  // Scene //

  anari::World getANARIWorld() const;

  // Graph Updates //

  template <typename T>
  void scheduleParameterUpdate(Parameter &p, T v);

  void update(GraphUpdateCallback cb = {});
  void sync();
  bool isReady() const;

  int numVisibleMappers() const;

  const TimeStamp &lastChange() const;

  // Utility //

  void print();

 private:
  void nodeChanged(Node *) override;

  std::vector<NodePtr> m_nodes;
  std::vector<Node *> m_primaryNodes;

  TimeStamp m_lastChange;
  bool m_needToUpdate{true};

  int m_numVisibleMappers{0};

  std::vector<DeferredParameterUpdateValue> m_parameterValueBuffer;

  mutable std::future<void> m_updateFuture;
  mutable interop::anari::ANARIScene m_scene;
};

///////////////////////////////////////////////////////////////////////////////
// Inlined definitions ////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

template <typename T, typename... Args>
inline T *ExecutionGraph::addNamedNode(const std::string &name, Args &&...args)
{
  static_assert(std::is_base_of<Node, T>::value,
      "ExecutionGraph::addNode() can only construct types derived from Node.");
  auto *node = new T(std::forward<Args>(args)...);
  if (!name.empty())
    node->setName(name);
  m_nodes.emplace_back(node);
  node->setObserver(this);
  if (node->isPrimary()) {
    m_primaryNodes.push_back(node);
    if (node->type() == NodeType::MAPPER)
      ((MapperNode *)node)->addMapperToScene(m_scene, {});
  }
  return node;
}

template <typename T, typename... Args>
inline T *ExecutionGraph::addNode(Args &&...args)
{
  return addNamedNode<T, Args...>("", std::forward<Args>(args)...);
}

template <typename T>
inline void ExecutionGraph::scheduleParameterUpdate(Parameter &p, T v)
{
  m_parameterValueBuffer.push_back({&p, ParameterRawValue{v}});
}

} // namespace graph
} // namespace vtkm