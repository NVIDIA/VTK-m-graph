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

#include "ExecutionGraph.h"
// std
#include <algorithm>

namespace vtkm_anari {
namespace graph {

// ExecutionGraph definitions /////////////////////////////////////////////////

ExecutionGraph::ExecutionGraph(anari::Device d) : m_scene(d) {}

ExecutionGraph::~ExecutionGraph()
{
  m_mapperNodes.clear(); // NOTE: destroy mapper nodes before graph
}

ActorNode *ExecutionGraph::addActorNode()
{
  m_actorNodes.emplace_back(std::make_unique<ActorNode>());
  return m_actorNodes.back().get();
}

size_t ExecutionGraph::getNumberOfNodes() const
{
  return getNumberOfSourceNodes() + getNumberOfFilterNodes()
      + getNumberOfActorNodes() + getNumberOfMapperNodes();
}

size_t ExecutionGraph::getNumberOfSourceNodes() const
{
  return m_sourceNodes.size();
}

size_t ExecutionGraph::getNumberOfFilterNodes() const
{
  return m_filterNodes.size();
}

size_t ExecutionGraph::getNumberOfActorNodes() const
{
  return m_actorNodes.size();
}

size_t ExecutionGraph::getNumberOfMapperNodes() const
{
  return m_mapperNodes.size();
}

SourceNode *ExecutionGraph::getSourceNode(size_t i) const
{
  return m_sourceNodes[i].get();
}

FilterNode *ExecutionGraph::getFilterNode(size_t i) const
{
  return m_filterNodes[i].get();
}

ActorNode *ExecutionGraph::getActorNode(size_t i) const
{
  return m_actorNodes[i].get();
}

MapperNode *ExecutionGraph::getMapperNode(size_t i) const
{
  return m_mapperNodes[i].get();
}

void ExecutionGraph::removeSourceNode(int id)
{
  m_sourceNodes.erase(std::remove_if(m_sourceNodes.begin(),
                          m_sourceNodes.end(),
                          [&](auto &n) { return n->id() == id; }),
      m_sourceNodes.end());
}

void ExecutionGraph::removeFilterNode(int id)
{
  m_filterNodes.erase(std::remove_if(m_filterNodes.begin(),
                          m_filterNodes.end(),
                          [&](auto &n) { return n->id() == id; }),
      m_filterNodes.end());
}

void ExecutionGraph::removeActorNode(int id)
{
  m_actorNodes.erase(std::remove_if(m_actorNodes.begin(),
                         m_actorNodes.end(),
                         [&](auto &n) { return n->id() == id; }),
      m_actorNodes.end());
}

void ExecutionGraph::removeMapperNode(int id)
{
  m_mapperNodes.erase(std::remove_if(m_mapperNodes.begin(),
                          m_mapperNodes.end(),
                          [&](auto &n) { return n->id() == id; }),
      m_mapperNodes.end());
}

void ExecutionGraph::removeNode(int id)
{
  removeSourceNode(id);
  removeFilterNode(id);
  removeActorNode(id);
  removeMapperNode(id);
}

anari::World ExecutionGraph::getANARIWorld() const
{
  return m_scene.GetANARIWorld();
}

void ExecutionGraph::updateWorld()
{
  if (!m_needToUpdate)
    return;

  m_lastChange.renew();
  m_numVisibleMappers = 0;
  try {
    for (auto &mn : m_mapperNodes) {
      mn->update();
      if (mn->isVisible() && !mn->isMapperEmpty())
        m_numVisibleMappers++;
    }
  } catch (const std::exception &e) {
    printf("--Error thrown when evaluating graph--\n\n%s\n", e.what());
  }

  m_needToUpdate = false;
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
  printf("---Source Nodes---\n");
  for (auto &s : m_sourceNodes)
    printf("%s\n", s->name());
  printf("\n");
  printf("---Actor Nodes---\n");
  for (auto &a : m_actorNodes)
    printf("%s\n", a->name());
  printf("\n");
  printf("---Mapper Nodes---\n");
  for (auto &m : m_mapperNodes)
    printf("%s\n", m->name());
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

} // namespace graph
} // namespace vtkm_anari
