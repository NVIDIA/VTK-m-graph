// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "Node.h"
#include "FilterNode.h"
#include "SourceNode.h"
// std
#include <sstream>
#include <stack>

namespace viskores {
namespace graph {

static int g_nextNodeID{0};
static std::stack<int> g_freeNodes;
ID_FCNS(Node)

static std::vector<Node *> g_nodes;

Node::Node(bool primary) : m_id(nextNodeID()), m_primary(primary)
{
  if (g_nodes.empty())
    g_nodes.resize(256, nullptr);
  g_nodes[id()] = this;
}

Node::~Node()
{
  g_nodes[id()] = nullptr;
  g_freeNodes.push(id());
}

const char *Node::name() const
{
  return m_name.c_str();
}

const char *Node::uniqueName() const
{
  if (m_uniqueName.empty())
    m_uniqueName = std::string(kind()) + std::to_string(id());
  return m_uniqueName.c_str();
}

const char *Node::summary() const
{
  return m_summary.empty() ? "<no summary>" : m_summary.c_str();
}

int Node::id() const
{
  return m_id;
}

bool Node::isPrimary() const
{
  return m_primary;
}

void Node::setName(std::string newName)
{
  m_name = newName;
}

size_t Node::numInput() const
{
  return 0;
}

InPort *Node::inputBegin()
{
  return nullptr;
}

InPort *Node::inputEnd()
{
  return inputBegin() + numInput();
}

InPort *Node::input(const char *name)
{
  auto param = std::find_if(inputBegin(), inputEnd(), [&](InPort &p) {
    return !std::strcmp(name, p.name());
  });
  return param == inputEnd() ? nullptr : &(*param);
}

size_t Node::numOutput() const
{
  return 0;
}

OutPort *Node::outputBegin()
{
  return nullptr;
}

OutPort *Node::outputEnd()
{
  return outputBegin() + numOutput();
}

OutPort *Node::output(const char *name)
{
  auto param = std::find_if(outputBegin(), outputEnd(), [&](OutPort &p) {
    return !std::strcmp(name, p.name());
  });
  return param == outputEnd() ? nullptr : &(*param);
}

Parameter *Node::parametersBegin()
{
  return m_parameters.data();
}

Parameter *Node::parametersEnd()
{
  return parametersBegin() + m_parameters.size();
}

Parameter *Node::parameter(const char *name)
{
  auto param = std::find_if(m_parameters.begin(),
      m_parameters.end(),
      [&](Parameter &p) { return !std::strcmp(name, p.name()); });
  return param == m_parameters.end() ? nullptr : &(*param);
}

size_t Node::numParameters() const
{
  return m_parameters.size();
}

Node *Node::fromID(int id)
{
  return id != INVALID_ID ? g_nodes[id] : nullptr;
}

Parameter *Node::addParameter(Parameter p)
{
  m_parameters.push_back(p);
  return &m_parameters.back();
}

void Node::notifyObserver()
{
  if (m_observer)
    m_observer->nodeChanged(this);
}

void Node::markChanged()
{
  m_lastChanged.renew();
  notifyObserver();
  for (auto *p = outputBegin(); p != outputEnd(); p++)
    for (auto *c = p->connectionsBegin(); c != p->connectionsEnd(); c++)
      InPort::fromID(*c)->node()->markChanged();
}

void Node::markUpdated()
{
  m_lastUpdated.renew();
}

bool Node::needsUpdate()
{
  return m_lastUpdated <= m_lastChanged;
}

void Node::setSummaryText(std::string str)
{
  m_summary = str;
}

void Node::setObserver(NodeObserver *observer)
{
  m_observer = observer;
}

template <typename T>
static T getPortValue(InPort *p)
{
  if (!p || !p->isConnected())
    return {};
  else {
    p->other()->node()->update();
    p->markValueReceived();
    return p->other()->value().Value<T>();
  }
}

cont::DataSet Node::getDataSetFromPort(InPort *p)
{
  auto ds = getPortValue<cont::DataSet>(p);
  p->selector()->setFieldNames(ds);
  return ds;
}

cont::UnknownCellSet Node::getCellSetFromPort(InPort *p)
{
  return getPortValue<cont::UnknownCellSet>(p);
}

cont::CoordinateSystem Node::getCoordinateSystemFromPort(InPort *p)
{
  return getPortValue<cont::CoordinateSystem>(p);
}

cont::Field Node::getFieldFromPort(InPort *p)
{
  return getPortValue<cont::Field>(p);
}

std::string getSummaryString(cont::DataSet d)
{
  std::stringstream ss;
  d.PrintSummary(ss);
  return ss.str();
}

} // namespace graph
} // namespace viskores
