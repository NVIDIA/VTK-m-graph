// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "Port.h"
#include "Node.h"
// std
#include <algorithm>
#include <random>
#include <stack>

namespace vtkm {
namespace graph {

static int g_nextInPortID{1};
static int g_nextOutPortID{1};
static std::stack<int> g_freeInPorts;
static std::stack<int> g_freeOutPorts;

ID_FCNS(InPort)
ID_FCNS(OutPort)

static std::vector<InPort *> g_inPorts;
static std::vector<OutPort *> g_outPorts;

Port::Port(PortType type, std::string name, Node *node)
    : m_type(type), m_name(name), m_node(node->id())
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
  return Node::fromID(m_node);
}

Port *Port::fromID(int id)
{
  if (graph::isInputPortID(id))
    return graph::InPort::fromID(id);
  else
    return graph::OutPort::fromID(id);
}

// InPort //

InPort::InPort(PortType type, std::string name, Node *node)
    : Port(type, name, node), m_id(nextInPortID())
{
  if (g_inPorts.empty())
    g_inPorts.resize(256, nullptr);
  updateAddress();
}

InPort::~InPort()
{
  if (m_id == INVALID_ID)
    return;
  disconnect();
  g_inPorts[id()] = nullptr;
  g_freeInPorts.push(id());
}

InPort::InPort(InPort &&o)
{
  std::swap(m_type, o.m_type);
  std::swap(m_name, o.m_name);
  std::swap(m_node, o.m_node);
  std::swap(m_connection, o.m_connection);
  std::swap(m_id, o.m_id);
  updateAddress();
}

InPort &InPort::operator=(InPort &&o)
{
  std::swap(m_type, o.m_type);
  std::swap(m_name, o.m_name);
  std::swap(m_node, o.m_node);
  std::swap(m_connection, o.m_connection);
  std::swap(m_id, o.m_id);
  updateAddress();
  return *this;
}

int InPort::id() const
{
  return m_id;
}

bool InPort::isConnected() const
{
  return m_connection != INVALID_ID;
}

bool InPort::connect(OutPort *from)
{
  if (from->type() != type())
    return false;
  disconnect();
  m_connection = from->id();
  node()->markChanged();
  return true;
}

void InPort::disconnect()
{
  if (!isConnected())
    return;
  auto *op = other();
  if (op)
    op->disconnect(this);
  m_connection = INVALID_ID;
  auto *n = node();
  if (n)
    n->markChanged();
}

OutPort *InPort::other() const
{
  return OutPort::fromID(m_connection);
}

const FieldSelector *InPort::cselector() const
{
  return type() == PortType::DATASET ? &m_selector : nullptr;
}

FieldSelector *InPort::selector()
{
  node()->markChanged();
  return type() == PortType::DATASET ? &m_selector : nullptr;
}

void InPort::markValueReceived()
{
  m_valueLastReceived.renew();
}

bool InPort::connectionHasNewValue() const
{
  return isConnected() && m_valueLastReceived < other()->valueLastChanged();
}

InPort *InPort::fromID(int id)
{
  return id != INVALID_ID ? g_inPorts[id] : nullptr;
}

void InPort::updateAddress()
{
  g_inPorts[id()] = this;
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
  updateAddress();
}

OutPort::~OutPort()
{
  if (m_id == INVALID_ID)
    return;
  auto i = indexFromOutPortID(id());
  g_outPorts[i] = nullptr;
  g_freeOutPorts.push(i);
  disconnectAllDownstreamPorts();
}

OutPort::OutPort(OutPort &&o)
{
  std::swap(m_type, o.m_type);
  std::swap(m_name, o.m_name);
  std::swap(m_node, o.m_node);
  std::swap(m_value, o.m_value);
  std::swap(m_connections, o.m_connections);
  std::swap(m_id, o.m_id);
  updateAddress();
}

OutPort &OutPort::operator=(OutPort &&o)
{
  std::swap(m_type, o.m_type);
  std::swap(m_name, o.m_name);
  std::swap(m_node, o.m_node);
  std::swap(m_value, o.m_value);
  std::swap(m_connections, o.m_connections);
  std::swap(m_id, o.m_id);
  updateAddress();
  return *this;
}

int OutPort::id() const
{
  return (m_id << 8) & 0xFF00;
}

bool OutPort::connect(InPort *from)
{
  if (from->type() != type())
    return false;
  m_connections.push_back(from->id());
  return true;
}

void OutPort::disconnect(InPort *from)
{
  m_connections.erase(
      std::remove(m_connections.begin(), m_connections.end(), from->id()),
      m_connections.end());
}

void OutPort::disconnectAllDownstreamPorts()
{
  for (auto c : m_connections)
    InPort::fromID(c)->disconnect();
}

int *OutPort::connectionsBegin()
{
  return m_connections.empty() ? nullptr : m_connections.data();
}

int *OutPort::connectionsEnd()
{
  return connectionsBegin() + m_connections.size();
}

PortValue OutPort::value()
{
  return m_value;
}

const TimeStamp &OutPort::valueLastChanged() const
{
  return m_valueLastChanged;
}

void OutPort::unsetValue()
{
  m_value.Reset();
}

OutPort *OutPort::fromID(int id)
{
  return id != INVALID_ID ? g_outPorts[indexFromOutPortID(id)] : nullptr;
}

void OutPort::updateAddress()
{
  g_outPorts[indexFromOutPortID(id())] = this;
}

// connect() //////////////////////////////////////////////////////////////////

bool connect(OutPort *from, InPort *to)
{
#if 0
  printf("Connecting ports: %s(%i) <--> %s(%i)\n",
      portTypeString(from->type()),
      from->id(),
      portTypeString(to->type()),
      to->id());
#endif

  return from->connect(to) && to->connect(from);
}

bool isInputPortID(int id)
{
  return id & 0x00FF;
}

const char *portTypeString(PortType type)
{
  switch (type) {
  case PortType::ACTOR:
    return "actor";
  case PortType::DATASET:
    return "dataset";
  case PortType::COORDINATE_SYSTEM:
    return "coordinate_system";
  case PortType::CELLSET:
    return "cellset";
  case PortType::FIELD:
    return "field";
  case PortType::UNKNOWN:
  default:
    break;
  }
  return "<unknown>";
}

} // namespace graph
} // namespace vtkm