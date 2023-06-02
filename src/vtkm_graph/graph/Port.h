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

// std
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
// vtk-m
#include <vtkm/interop/anari/ANARIActor.h>

#include "../utility/Any.h"
#include "FieldSelector.h"
#include "TimeStamp.h"

#include "vtkm_graph_export.h"

namespace vtkm {
namespace graph {

enum class PortType
{
  DATASET,
  ACTOR,
  COORDINATE_SYSTEM,
  CELLSET,
  FIELD,
  UNKNOWN
};

using PortValue = Any;

static constexpr int INVALID_ID = -1;

struct Node;

struct VTKM_GRAPH_EXPORT Port
{
  Port(PortType type, std::string name, Node *node);
  virtual ~Port() = default;

  PortType type() const;
  const char *name() const;
  Node *node();

  virtual int id() const = 0;

  // Not copyable
  Port(const Port &) = delete;
  Port &operator=(const Port &) = delete;

  Port(Port &&) = default;
  Port &operator=(Port &&) = default;

  static Port *fromID(int id);

 protected:
  Port() = default;

  PortType m_type{PortType::UNKNOWN};
  std::string m_name;
  int m_node{INVALID_ID};
};

struct OutPort;

struct VTKM_GRAPH_EXPORT InPort : public Port
{
  InPort() = default;
  InPort(PortType type, std::string name, Node *node);
  ~InPort() override;

  InPort(InPort &&);
  InPort &operator=(InPort &&);

  int id() const override;

  bool isConnected() const;

  bool connect(OutPort *from);
  void disconnect();
  OutPort *other() const;

  const FieldSelector *cselector() const;
  FieldSelector *selector();

  void markValueReceived();
  bool connectionHasNewValue() const;

  static InPort *fromID(int id);

 private:
  void updateAddress();

  int m_connection{INVALID_ID};
  int m_id{INVALID_ID};

  TimeStamp m_valueLastReceived{};

  FieldSelector m_selector;
};

struct VTKM_GRAPH_EXPORT OutPort : public Port
{
  OutPort() = default;
  OutPort(PortType type, std::string name, Node *node);
  ~OutPort() override;

  OutPort(OutPort &&);
  OutPort &operator=(OutPort &&);

  int id() const override;

  bool connect(InPort *from);
  void disconnect(InPort *from);
  void disconnectAllDownstreamPorts();

  int *connectionsBegin();
  int *connectionsEnd();

  template <typename T>
  void setValue(const T &v);
  PortValue value();
  const TimeStamp &valueLastChanged() const;
  void unsetValue();

  static OutPort *fromID(int id);

 private:
  void updateAddress();

  PortValue m_value;
  TimeStamp m_valueLastChanged{};
  std::vector<int> m_connections;
  int m_id{INVALID_ID};
};

VTKM_GRAPH_EXPORT bool connect(OutPort *from, InPort *to);
VTKM_GRAPH_EXPORT bool isInputPortID(int id);
VTKM_GRAPH_EXPORT const char *portTypeString(PortType type);

// Inlined definitions ////////////////////////////////////////////////////////

template <typename T>
inline void OutPort::setValue(const T &v)
{
  m_value = v;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

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
        throw std::runtime_error("exhausted number of graph objects");         \
      return v;                                                                \
    }                                                                          \
  }

} // namespace graph
} // namespace vtkm