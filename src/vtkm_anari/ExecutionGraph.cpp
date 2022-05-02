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
#include <stack>
// vtk-m
#include <vtkm/source/Tangle.h>

#include "vtkm_anari/ANARIMapperVolume.h"

namespace vtkm_anari::graph {

static int g_nextInPortID{0};
static int g_nextOutPortID{0};
static int g_nextNodeID{0};
static std::stack<int> g_freeInPorts;
static std::stack<int> g_freeOutPorts;
static std::stack<int> g_freeNodes;

#define ID_FCNS(type)                                                          \
  int next##type##ID()                                                         \
  {                                                                            \
    if (!g_free##type##s.empty()) {                                            \
      auto id = g_free##type##s.top();                                         \
      g_free##type##s.pop();                                                   \
      return id;                                                               \
    } else {                                                                   \
      auto v = g_next##type##ID++;                                             \
      if (v >= 255)                                                            \
        throw std::runtime_error("cannot make more than 255 graph objects");   \
      return v;                                                                \
    }                                                                          \
  }

ID_FCNS(InPort)
ID_FCNS(OutPort)
ID_FCNS(Node)

static std::vector<InPort *> g_inPorts;
static std::vector<OutPort *> g_outPorts;

///////////////////////////////////////////////////////////////////////////////
// Port ///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Port //

Port::Port(PortType type, std::string name, Node *node)
    : m_type(type), m_name(name), m_node(node)
{}

PortType Port::type() const
{
  return m_type;
}

const char *Port::name() const
{
  return m_name.c_str();
}

Node *Port::node()
{
  return m_node;
}

// InPort //

InPort::InPort(PortType type, std::string name, Node *node)
    : Port(type, name, node), m_id(nextInPortID())
{
  if (g_inPorts.empty())
    g_inPorts.resize(256);
  g_inPorts[id()] = this;
}

InPort::~InPort()
{
  disconnect();
  g_inPorts[id()] = nullptr;
  g_freeInPorts.push(id());
}

int InPort::id() const
{
  return m_id;
}

bool InPort::isConnected() const
{
  return m_connection != nullptr;
}

bool InPort::connect(OutPort *from)
{
  if (!from || from->type() != type())
    return false;
  disconnect();
  m_connection = from;
  return true;
}

void InPort::disconnect()
{
  if (!isConnected())
    return;
  m_connection->disconnect(this);
  m_connection = nullptr;
}

OutPort *InPort::other() const
{
  return m_connection;
}

InPort *InPort::fromID(int id)
{
  return g_inPorts[id];
}

// OutPort //

OutPort::OutPort(PortType type, std::string name, Node *node)
    : Port(type, name, node), m_id(nextOutPortID())
{
  if (g_outPorts.empty())
    g_outPorts.resize(256);
  g_outPorts[id()] = this;
}

OutPort::~OutPort()
{
  g_outPorts[id()] = nullptr;
  g_freeOutPorts.push(id());
  while (!m_connections.empty()) {
    m_connections.back()->disconnect();
    m_connections.pop_back();
  }
}

int OutPort::id() const
{
  return m_id;
}

bool OutPort::connect(InPort *from)
{
  if (!from || from->type() != type())
    return false;
  m_connections.push_back(from);
  return true;
}

void OutPort::disconnect(InPort *from)
{
  if (!from)
    return;
  m_connections.erase(
      std::remove(m_connections.begin(), m_connections.end(), from),
      m_connections.end());
}

OutPort *OutPort::fromID(int id)
{
  return g_outPorts[id];
}

// connect() //////////////////////////////////////////////////////////////////

bool connect(OutPort *from, InPort *to)
{
  if (!from->connect(to))
    return false;
  return to->connect(from);
}

///////////////////////////////////////////////////////////////////////////////
// Node ///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

Node::Node() : m_id(nextNodeID()) {}

Node::~Node()
{
  g_freeNodes.push(id());
}

const char *Node::name() const
{
  if (m_name.empty())
    m_name = std::string(kind()) + '_' + std::to_string(id());
  return m_name.c_str();
}

int Node::id() const
{
  return m_id;
}

InPort *Node::input(const char *)
{
  return nullptr;
}

OutPort *Node::output(const char *)
{
  return nullptr;
}

// SourceNode //

OutPort *SourceNode::output(const char *name)
{
  if (!std::strcmp(name, m_datasetPort.name()))
    return &m_datasetPort;
  return nullptr;
}

bool SourceNode::isValid() const
{
  return true;
}

// TangleSourceNode //

const char *TangleSourceNode::kind() const
{
  return "TangleSource";
}

vtkm::cont::DataSet TangleSourceNode::dataset()
{
  return vtkm::source::Tangle(vtkm::Id3{64}).Execute();
}

// ActorNode //

const char *ActorNode::kind() const
{
  return "Actor";
}

InPort *ActorNode::input(const char *name)
{
  if (!std::strcmp(name, m_datasetPort.name()))
    return &m_datasetPort;
  return nullptr;
}

OutPort *ActorNode::output(const char *name)
{
  if (!std::strcmp(name, m_actorPort.name()))
    return &m_actorPort;
  return nullptr;
}

bool ActorNode::isValid() const
{
  return m_datasetPort.isConnected();
}

ANARIActor ActorNode::makeActor(vtkm::cont::DataSet ds)
{
  return ANARIActor(ds.GetCellSet(), ds.GetCoordinateSystem(), ds.GetField(0));
}

// MapperNode //

InPort *MapperNode::input(const char *name)
{
  if (!std::strcmp(name, m_actorPort.name()))
    return &m_actorPort;
  return nullptr;
}

bool MapperNode::isValid() const
{
  return m_actorPort.isConnected();
}

// VolumeMapperNode //

const char *VolumeMapperNode::kind() const
{
  return "VolumeMapper";
}

void VolumeMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperVolume(scene.GetDevice(), a));
}

///////////////////////////////////////////////////////////////////////////////
// Graph //////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

ExecutionGraph::ExecutionGraph(anari::Device d) : m_scene(d) {}

ActorNode *ExecutionGraph::addActorNode()
{
  m_actorNodes.emplace_back(std::make_unique<ActorNode>());
  return m_actorNodes.back().get();
}

size_t ExecutionGraph::getNumberOfSourceNodes() const
{
  return m_sourceNodes.size();
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

anari::World ExecutionGraph::getANARIWorld() const
{
  return m_scene.GetANARIWorld();
}

void ExecutionGraph::updateWorld()
{
  for (auto &mn : m_mapperNodes) {
    if (!mn->isValid())
      continue;

    auto *an = (ActorNode *)mn->input("actor")->other()->node();

    if (!an->isValid())
      continue;

    // TODO: filter node stack

    auto *sn = (SourceNode *)an->input("dataset")->other()->node();

    if (!sn->isValid())
      continue;

    auto d = sn->dataset();
    auto a = an->makeActor(d);
    mn->addMapperToScene(m_scene, a);
  }
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

} // namespace vtkm_anari::graph