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
#include <mutex>

namespace vtkm {
namespace graph {

using GraphUpdateCallback = std::function<void()>;

// NOTE: This governs the execution behavior of ExecutionGraph::update()
enum class GraphExecutionPolicy
{
  // clang-format off
  MAIN_THREAD_ONLY,   // All nodes updated on the calling thread (synchronous)
  FILTER_NODES_ASYNC, // Source/filter nodes executed asynchronously
  ALL_ASYNC           // All nodes executed asynchronously
            // clang-format on
};

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

  void update(GraphExecutionPolicy policy = GraphExecutionPolicy::ALL_ASYNC,
      GraphUpdateCallback cb = {});
  void sync();
  bool isReady() const;

  int numVisibleMappers() const;

  const TimeStamp &lastChange() const;

  // Utility //

  void print();

 private:
  void nodeChanged(Node *) override;
  bool needsToUpdate() const;
  void consumeParameters();

  using NodePtr = std::unique_ptr<Node>;
  std::vector<NodePtr> m_nodes;
  std::vector<Node *> m_primaryNodes;

  TimeStamp m_lastChange;
  bool m_needToUpdate{true};
  bool m_isUpdating{false};
  GraphExecutionPolicy m_currentPolicy{GraphExecutionPolicy::MAIN_THREAD_ONLY};

  int m_numVisibleMappers{0};

  std::vector<DeferredParameterUpdateValue> m_parameterValueBuffer;
  mutable std::mutex m_parameterBufferMutex;

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
  else
    node->setName(node->kind());
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
  std::lock_guard<std::mutex> guard(m_parameterBufferMutex);
  m_parameterValueBuffer.push_back({&p, ParameterRawValue{v}});
}

} // namespace graph
} // namespace vtkm
