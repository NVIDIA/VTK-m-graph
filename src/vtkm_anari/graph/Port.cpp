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

#include "Port.h"
// std
#include <algorithm>
#include <random>
#include <stack>

namespace vtkm_anari::graph {

static int g_nextInPortID{1};
static int g_nextOutPortID{1};
static std::stack<int> g_freeInPorts;
static std::stack<int> g_freeOutPorts;

ID_FCNS(InPort)
ID_FCNS(OutPort)

static std::vector<InPort *> g_inPorts;
static std::vector<OutPort *> g_outPorts;

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

} // namespace vtkm_anari::graph