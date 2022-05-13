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

#pragma once

#include "Parameter.h"
#include "Port.h"

namespace vtkm_anari::graph {

struct Node;
struct ExecutionGraph;

struct NodeObserver
{
  virtual void nodeChanged(Node *) {}
};

enum class NodeType
{
  SOURCE,
  FILTER,
  ACTOR,
  MAPPER
};

struct VTKM_ANARI_EXPORT Node : ParameterObserver
{
  Node();
  virtual ~Node();

  virtual bool isValid() const = 0;

  const char *name() const;
  virtual NodeType type() const = 0;
  virtual const char *kind() const = 0;
  int id() const;

  virtual InPort *input(const char *name);
  virtual OutPort *output(const char *name);

  Parameter *parametersBegin();
  Parameter *parametersEnd();
  Parameter *parameter(const char *name);

 protected:
  Parameter *addParameter(Parameter p);
  void notifyObserver();

 private:
  friend struct ExecutionGraph;
  void setObserver(NodeObserver *observer);

  mutable std::string m_name;
  int m_id{-1};
  std::vector<Parameter> m_parameters;
  NodeObserver *m_observer{nullptr};
};

} // namespace vtkm_anari::graph
