// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "ActorNode.h"
#include "FilterNode.h"
#include "SourceNode.h"

namespace viskores {
namespace graph {

ActorNode::ActorNode()
{
  selector().setFieldNames({});
}

ActorNode::~ActorNode()
{
  m_datasetPort.disconnect();
  m_actorPort.disconnectAllDownstreamPorts();
}

const char *ActorNode::kind() const
{
  return "Actor";
}

InPort *ActorNode::inputBegin()
{
  return &m_datasetPort;
}

size_t ActorNode::numInput() const
{
  return 1;
}

OutPort *ActorNode::outputBegin()
{
  return &m_actorPort;
}

size_t ActorNode::numOutput() const
{
  return 1;
}

NodeType ActorNode::type() const
{
  return NodeType::ACTOR;
}

bool ActorNode::isValid() const
{
  return m_datasetPort.isConnected();
}

const FieldSelector &ActorNode::cselector() const
{
  return m_selector;
}

FieldSelector &ActorNode::selector()
{
  markChanged();
  return m_selector;
}

void ActorNode::update()
{
  if (!needsUpdate())
    return;
  m_actorPort.setValue(makeActor(getDataSetFromPort(&m_datasetPort)));
  markUpdated();
}

interop::anari::ANARIActor ActorNode::makeActor(cont::DataSet ds)
{
  selector().setFieldNames(ds);
#if 1
  const auto &s = cselector();
  size_t primaryFieldIdx = s.currentField();
  primaryFieldIdx = primaryFieldIdx > 3 ? 0 : primaryFieldIdx;

  auto getField = [&](size_t i) -> cont::Field {
    if (s.numFields() <= i)
      return {};
    std::string fieldName = s.fieldName(i);
    return fieldName == "[none]" ? cont::Field{} : ds.GetField(fieldName);
  };

  cont::UnknownCellSet cellSet = ds.GetCellSet();
  cont::CoordinateSystem coordSys;
  if (ds.GetNumberOfCoordinateSystems() > 0)
    coordSys = ds.GetCoordinateSystem();

  auto actor = interop::anari::ANARIActor(
      cellSet, coordSys, getField(0), getField(1), getField(2), getField(3));
  actor.SetPrimaryFieldIndex(primaryFieldIdx);
#else
  auto actor = interop::anari::ANARIActor(ds);
  actor.SetPrimaryFieldIndex(m_currentField);
#endif
  return actor;
}

} // namespace graph
} // namespace viskores
