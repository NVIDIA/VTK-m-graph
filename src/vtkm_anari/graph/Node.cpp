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

#include "Node.h"
#include "FilterNode.h"
#include "SourceNode.h"
// std
#include <sstream>
#include <stack>

namespace vtkm_anari {
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
  if (m_name.empty())
    m_name = std::string(kind()) + '_' + std::to_string(id());
  return m_name.c_str();
}

const char *Node::title() const
{
  if (m_title.empty())
    m_title = std::string(kind());
  return m_title.c_str();
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

void Node::setTitle(const char *newTitle)
{
  m_title = newTitle;
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

vtkm::cont::DataSet Node::getDataSetFromPort(InPort *p)
{
  if (!p || !p->isConnected())
    return {};

  auto *node = p->other()->node();
  if (node->type() == NodeType::FILTER)
    return ((FilterNode *)node)->dataset();
  else if (node->type() == NodeType::SOURCE)
    return ((SourceNode *)node)->dataset();

  return {};
}

std::string getSummaryString(vtkm::cont::DataSet d)
{
  std::stringstream ss;
  d.PrintSummary(ss);
  return ss.str();
}

} // namespace graph
} // namespace vtkm_anari
