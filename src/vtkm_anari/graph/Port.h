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

#include "../ANARIExports.h"

namespace vtkm_anari::graph {

enum class PortType
{
  DATASET,
  ACTOR,
  UNKNOWN
};

struct Node;

struct VTKM_ANARI_EXPORT Port
{
  Port(PortType type, std::string name, Node *node);
  virtual ~Port() = default;

  PortType type() const;
  const char *name() const;
  Node *node();

  // Not copyable or movable
  Port(const Port &) = delete;
  Port(Port &&) = delete;
  Port &operator=(const Port &) = delete;
  Port &operator=(Port &&) = delete;

 private:
  PortType m_type{PortType::UNKNOWN};
  std::string m_name;
  Node *m_node{nullptr};
};

struct OutPort;

struct VTKM_ANARI_EXPORT InPort : public Port
{
  InPort(PortType type, std::string name, Node *node);
  ~InPort() override;

  int id() const;

  bool isConnected() const;

  bool connect(OutPort *from);
  void disconnect();
  OutPort *other() const;

  static InPort *fromID(int id);

 private:
  OutPort *m_connection{nullptr};
  int m_id{-1};
};

struct VTKM_ANARI_EXPORT OutPort : public Port
{
  OutPort(PortType type, std::string name, Node *node);
  ~OutPort() override;

  int id() const;

  bool connect(InPort *from);
  void disconnect(InPort *from);
  void disconnectAllDownstreamPorts();

  static OutPort *fromID(int id);

 private:
  std::vector<InPort *> m_connections;
  int m_id{-1};
};

VTKM_ANARI_EXPORT bool connect(OutPort *from, InPort *to);

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
      if (v > 255)                                                             \
        throw std::runtime_error("cannot make more than 255 graph objects");   \
      return v;                                                                \
    }                                                                          \
  }

} // namespace vtkm_anari::graph
