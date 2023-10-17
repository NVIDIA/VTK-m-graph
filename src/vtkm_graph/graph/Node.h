// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Parameter.h"
#include "Port.h"
#include "TimeStamp.h"
// vtk-m
#include <vtkm/interop/anari/ANARIActor.h>

namespace vtkm {
namespace graph {

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
  MAPPER,
  UTILITY
};

struct VTKM_GRAPH_EXPORT Node : ParameterObserver
{
  Node(bool primary = false);
  virtual ~Node();

  virtual bool isValid() const = 0;

  const char *name() const;
  const char *summary() const;
  virtual NodeType type() const = 0;
  virtual const char *kind() const = 0;
  int id() const;
  bool isPrimary() const;

  void setName(std::string newName);

  virtual size_t numInput() const;
  virtual InPort *inputBegin();
  InPort *inputEnd();
  InPort *input(const char *name);

  virtual size_t numOutput() const;
  virtual OutPort *outputBegin();
  OutPort *outputEnd();
  OutPort *output(const char *name);

  Parameter *parametersBegin();
  Parameter *parametersEnd();
  Parameter *parameter(const char *name);
  size_t numParameters() const;

  static Node *fromID(int id);

  virtual void update() = 0;

 protected:
  Parameter *addParameter(Parameter p);
  void notifyObserver();

  void markChanged();
  void markUpdated();
  virtual bool needsUpdate();

  void setSummaryText(std::string str);

  cont::DataSet getDataSetFromPort(InPort *p);
  cont::UnknownCellSet getCellSetFromPort(InPort *p);
  cont::CoordinateSystem getCoordinateSystemFromPort(InPort *p);
  cont::Field getFieldFromPort(InPort *p);

 private:
  friend struct ExecutionGraph;
  friend struct InPort;

  void setObserver(NodeObserver *observer);

  mutable std::string m_name;
  std::string m_summary;
  int m_id{INVALID_ID};
  bool m_primary{false};
  std::vector<Parameter> m_parameters;
  NodeObserver *m_observer{nullptr};
  TimeStamp m_lastUpdated;
  TimeStamp m_lastChanged;
};

VTKM_GRAPH_EXPORT std::string getSummaryString(cont::DataSet d);

} // namespace graph
} // namespace vtkm
