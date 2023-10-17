// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "graph/ActorNode.h"
#include "graph/FilterNode.h"
#include "graph/MapperNode.h"
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

  using NodePtr = std::unique_ptr<Node>;
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
