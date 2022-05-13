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
// std
#include <stack>

static int g_nextNodeID{0};
static std::stack<int> g_freeNodes;
ID_FCNS(Node)

namespace vtkm_anari::graph {

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

void Node::setObserver(NodeObserver *observer)
{
  m_observer = observer;
}

} // namespace vtkm_anari::graph
