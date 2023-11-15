// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "ExecutionGraph.h"
// std
#include <algorithm>
#include <chrono>

namespace vtkm {
namespace graph {

// Helper functions ///////////////////////////////////////////////////////////

template <typename R>
static bool is_ready(const std::future<R> &f)
{
  return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

// ExecutionGraph definitions /////////////////////////////////////////////////

ExecutionGraph::ExecutionGraph(anari::Device d) : m_scene(d) {}

ExecutionGraph::~ExecutionGraph()
{
  sync();
  m_nodes.clear();
}

size_t ExecutionGraph::getNumberOfNodes() const
{
  return m_nodes.size();
}

Node *ExecutionGraph::getNode(size_t i) const
{
  return m_nodes[i].get();
}

void ExecutionGraph::removeNode(int id)
{
  m_primaryNodes.erase(std::remove_if(m_primaryNodes.begin(),
                           m_primaryNodes.end(),
                           [&](auto &n) { return n->id() == id; }),
      m_primaryNodes.end());
  m_nodes.erase(std::remove_if(m_nodes.begin(),
                    m_nodes.end(),
                    [&](auto &n) { return n->id() == id; }),
      m_nodes.end());
}

anari::World ExecutionGraph::getANARIWorld() const
{
  return m_scene.GetANARIWorld();
}

void ExecutionGraph::update(GraphUpdateCallback _cb)
{
  if (m_isUpdating)
    return;

  if (!needsToUpdate())
    return;

  m_isUpdating = true;

  m_updateFuture = std::async([&, cb = std::move(_cb)]() {
    while (needsToUpdate()) {
      m_numVisibleMappers = 0;
      consumeParameters();

      try {
        for (auto *n : m_primaryNodes) {
          n->update();
          if (n->type() == NodeType::MAPPER) {
            auto *mn = (MapperNode *)n;
            if (!mn->isMapperEmpty())
              m_numVisibleMappers++;
          }
        }
      } catch (const std::exception &e) {
        printf("--Error thrown when evaluating graph--\n\n%s\n", e.what());
      }

      m_needToUpdate = false;
    }

    m_isUpdating = false;

    if (cb)
      cb();
  });
}

void ExecutionGraph::sync()
{
  if (m_updateFuture.valid())
    m_updateFuture.get();
}

bool ExecutionGraph::isReady() const
{
  return !m_updateFuture.valid() || is_ready(m_updateFuture);
}

int ExecutionGraph::numVisibleMappers() const
{
  return m_numVisibleMappers;
}

const TimeStamp &ExecutionGraph::lastChange() const
{
  return m_lastChange;
}

void ExecutionGraph::print()
{
  printf("\n");
  printf("---Nodes---\n");
  for (auto &s : m_nodes)
    printf("%s\n", s->uniqueName());
  printf("\n");
  printf("---Mapper Nodes---\n");
  for (auto &m : m_primaryNodes)
    printf("%s\n", m->uniqueName());
  printf("\n");

  printf("mappers added to scene: {'%s'", m_scene.GetMapper(0).GetName());
  for (size_t i = 1; i < m_scene.GetNumberOfMappers(); i++)
    printf(",'%s'", m_scene.GetMapper(i).GetName());
  printf("}\n");
}

void ExecutionGraph::nodeChanged(Node *)
{
  m_needToUpdate = true;
}

bool ExecutionGraph::needsToUpdate() const
{
  std::lock_guard<std::mutex> guard(m_parameterBufferMutex);
  return !m_parameterValueBuffer.empty() || m_needToUpdate;
}

void ExecutionGraph::consumeParameters()
{
  std::lock_guard<std::mutex> guard(m_parameterBufferMutex);

  for (auto &dp : m_parameterValueBuffer)
    dp.p->setRawValue(std::move(dp.v));

  m_parameterValueBuffer.clear();
  m_lastChange.renew();
}

} // namespace graph
} // namespace vtkm
