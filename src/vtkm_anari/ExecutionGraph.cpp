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
  m_mapperNodes.erase(std::remove_if(m_mapperNodes.begin(),
                          m_mapperNodes.end(),
                          [&](auto &n) { return n->id() == id; }),
      m_mapperNodes.end());
  m_nodes.erase(std::remove_if(m_nodes.begin(),
                    m_nodes.end(),
                    [&](auto &n) { return n->id() == id; }),
      m_nodes.end());
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
    for (auto *mn : m_mapperNodes) {
      if (!mn->isVisible())
        continue;
      mn->update();
      if (!mn->isMapperEmpty())
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
  printf("---Nodes---\n");
  for (auto &s : m_nodes)
    printf("%s\n", s->name());
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
