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
#include <random>
#include <stack>
// vtk-m
#include <vtkm/cont/DataSetBuilderExplicit.h>
#include <vtkm/filter/CellAverage.h>
#include <vtkm/filter/CleanGrid.h>
#include <vtkm/filter/Contour.h>
#include <vtkm/filter/Gradient.h>
#include <vtkm/filter/PointAverage.h>
#include <vtkm/filter/VectorMagnitude.h>
#include <vtkm/source/Tangle.h>

#include "vtkm_anari/ANARIMapperGlyphs.h"
#include "vtkm_anari/ANARIMapperPoints.h"
#include "vtkm_anari/ANARIMapperTriangles.h"
#include "vtkm_anari/ANARIMapperVolume.h"

namespace vtkm_anari::graph {

static int g_nextInPortID{1};
static int g_nextOutPortID{1};
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
      if (v > 255)                                                             \
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
    g_inPorts.resize(256, nullptr);
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

static int indexFromOutPortID(int id)
{
  return (id >> 8) & 0x00FF;
}

OutPort::OutPort(PortType type, std::string name, Node *node)
    : Port(type, name, node), m_id(nextOutPortID())
{
  if (g_outPorts.empty())
    g_outPorts.resize(256, nullptr);
  g_outPorts[indexFromOutPortID(id())] = this;
}

OutPort::~OutPort()
{
  auto i = indexFromOutPortID(id());
  g_outPorts[i] = nullptr;
  g_freeOutPorts.push(i);
  disconnectAllDownstreamPorts();
}

int OutPort::id() const
{
  return (m_id << 8) & 0xFF00;
}

bool OutPort::connect(InPort *from)
{
  if (from->type() != type())
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

void OutPort::disconnectAllDownstreamPorts()
{
  for (auto *c : m_connections)
    c->disconnect();
}

OutPort *OutPort::fromID(int id)
{
  return g_outPorts[indexFromOutPortID(id)];
}

// connect() //////////////////////////////////////////////////////////////////

bool connect(OutPort *from, InPort *to)
{
#if 0
  printf("Connecting ports: %s(%i) <--> %s(%i)\n",
      from->type() == PortType::DATASET ? "DATASET" : "ACTOR",
      from->id(),
      to->type() == PortType::DATASET ? "DATASET" : "ACTOR",
      to->id());
#endif
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

SourceNode::~SourceNode()
{
  m_datasetPort.disconnectAllDownstreamPorts();
}

OutPort *SourceNode::output(const char *name)
{
  if (!std::strcmp(name, m_datasetPort.name()))
    return &m_datasetPort;
  return nullptr;
}

NodeType SourceNode::type() const
{
  return NodeType::SOURCE;
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

// RandomPointsSourceNode //

const char *RandomPointsSourceNode::kind() const
{
  return "RandomPointsSource";
}

vtkm::cont::DataSet RandomPointsSourceNode::dataset()
{
  constexpr int numSpheres = 1e4;

  std::mt19937 rng;
  rng.seed(0);
  std::normal_distribution<float> vert_dist(0.f, 4.f);

  vtkm::cont::DataSetBuilderExplicitIterative builder;

  for (int i = 0; i < numSpheres; i++) {
    builder.AddPoint(vert_dist(rng), vert_dist(rng), vert_dist(rng));
    builder.AddCell(vtkm::CELL_SHAPE_VERTEX);
    builder.AddCellPoint(i);
  }

  return builder.Create();
}

// FilterNode //

FilterNode::~FilterNode()
{
  m_datasetInPort.disconnect();
  m_datasetOutPort.disconnectAllDownstreamPorts();
}

InPort *FilterNode::input(const char *name)
{
  if (!std::strcmp(name, m_datasetInPort.name()))
    return &m_datasetInPort;
  return nullptr;
}

OutPort *FilterNode::output(const char *name)
{
  if (!std::strcmp(name, m_datasetOutPort.name()))
    return &m_datasetOutPort;
  return nullptr;
}

NodeType FilterNode::type() const
{
  return NodeType::FILTER;
}

bool FilterNode::isValid() const
{
  return m_datasetInPort.isConnected();
}

// ContourNode //

const char *ContourNode::kind() const
{
  return "Contour";
}

vtkm::cont::DataSet ContourNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::Range range;
  auto field = ds.GetField(0);
  field.GetRange(&range);
  const auto isovalue = range.Center();

  vtkm::filter::Contour filter;
  filter.SetIsoValue(isovalue);
  filter.SetActiveField(field.GetName());
  return filter.Execute(ds);
}

// GradientNode //

const char *GradientNode::kind() const
{
  return "Gradient";
}

vtkm::cont::DataSet GradientNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::Gradient filter;
  filter.SetActiveField(ds.GetField(0).GetName());
  filter.SetOutputFieldName("Gradient");
  return filter.Execute(ds);
}

// CleanGridNode //

const char *CleanGridNode::kind() const
{
  return "CleanGrid";
}

vtkm::cont::DataSet CleanGridNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::CleanGrid filter;
  return filter.Execute(ds);
}

// VectorMagnitudeNode //

const char *VectorMagnitudeNode::kind() const
{
  return "VectorMagnitude";
}

vtkm::cont::DataSet VectorMagnitudeNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::VectorMagnitude filter;
  filter.SetActiveField(ds.GetField(0).GetName());
  return filter.Execute(ds);
}

// PointAverageNode //

const char *PointAverageNode::kind() const
{
  return "PointAverage";
}

vtkm::cont::DataSet PointAverageNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::PointAverage filter;
  filter.SetActiveField(ds.GetField(0).GetName());
  return filter.Execute(ds);
}

// CellAverageNode //

const char *CellAverageNode::kind() const
{
  return "CellAverage";
}

vtkm::cont::DataSet CellAverageNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::CellAverage filter;
  filter.SetActiveField(ds.GetField(0).GetName());
  return filter.Execute(ds);
}

// ActorNode //

ActorNode::~ActorNode()
{
  m_datasetPort.disconnect();
  m_actorPort.disconnectAllDownstreamPorts();
}

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

NodeType ActorNode::type() const
{
  return NodeType::ACTOR;
}

bool ActorNode::isValid() const
{
  return m_datasetPort.isConnected();
}

ANARIActor ActorNode::makeActor(vtkm::cont::DataSet ds)
{
  if (ds.GetNumberOfFields() == 0)
    return ANARIActor(ds.GetCellSet(), ds.GetCoordinateSystem(), {});
  else
    return ANARIActor(
        ds.GetCellSet(), ds.GetCoordinateSystem(), ds.GetField(0));
}

// MapperNode //

MapperNode::~MapperNode()
{
  m_actorPort.disconnect();
}

InPort *MapperNode::input(const char *name)
{
  if (!std::strcmp(name, m_actorPort.name()))
    return &m_actorPort;
  return nullptr;
}

NodeType MapperNode::type() const
{
  return NodeType::MAPPER;
}

bool MapperNode::isValid() const
{
  return m_actorPort.isConnected();
}

bool MapperNode::isVisible() const
{
  return m_visible;
}

void MapperNode::setVisible(bool show)
{
  m_visible = show;
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

// TriangleMapperNode //

const char *TriangleMapperNode::kind() const
{
  return "TriangleMapper";
}

void TriangleMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperTriangles(scene.GetDevice(), a));
}

// PointMapperNode //

const char *PointMapperNode::kind() const
{
  return "PointMapper";
}

void PointMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperPoints(scene.GetDevice(), a));
}

// GlyphMapperNode //

const char *GlyphMapperNode::kind() const
{
  return "GlyphMapper";
}

void GlyphMapperNode::addMapperToScene(ANARIScene &scene, ANARIActor a)
{
  scene.AddMapper(ANARIMapperGlyphs(scene.GetDevice(), a));
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
  m_scene.RemoveAllMappers();

  for (auto &mn : m_mapperNodes) {
    std::stack<FilterNode *> filterNodes;

    if (!mn->isValid() || !mn->isVisible())
      continue;

    auto *an = (ActorNode *)mn->input("actor")->other()->node();

    if (!an->isValid())
      continue;

    bool foundInvalidFilter = false;

    auto *lastNode = an->input("dataset")->other()->node();
    while (lastNode->type() != NodeType::SOURCE) {
      if (!lastNode->isValid()) {
        foundInvalidFilter = true;
        break;
      }
      filterNodes.push((FilterNode *)lastNode);
      lastNode = lastNode->input("dataset")->other()->node();
    }

    if (foundInvalidFilter)
      continue;

    auto *sn = (SourceNode *)lastNode;

    if (!sn->isValid())
      continue;

    auto d = sn->dataset();
    while (!filterNodes.empty()) {
      auto *fn = filterNodes.top();
      d = fn->execute(d);
      filterNodes.pop();
    }
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
