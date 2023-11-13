// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../UtilityNode.h"

namespace vtkm {
namespace graph {

ExtractActorFieldRangeNode::ExtractActorFieldRangeNode() : UtilityNode(true)
{
  addParameter({this, "active", ParameterType::BOOL, true});
}

const char *ExtractActorFieldRangeNode::kind() const
{
  return "ExtractActorFieldRange";
}

void ExtractActorFieldRangeNode::parameterChanged(
    Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "active"))
      m_active = p->valueAs<bool>();
  }

  markChanged();
}

size_t ExtractActorFieldRangeNode::numInput() const
{
  return 1;
}

InPort *ExtractActorFieldRangeNode::inputBegin()
{
  return &m_actorPort;
}

void ExtractActorFieldRangeNode::setCallback(ValueRangeCallback cb)
{
  m_callback = cb;
}

bool ExtractActorFieldRangeNode::needsUpdate()
{
  return m_active && Node::needsUpdate();
}

void ExtractActorFieldRangeNode::update()
{
  if (!needsUpdate())
    return;

  if (!m_actorPort.isConnected()) {
    m_range = Range(0, 0);
    if (m_callback)
      m_callback(m_range);
    markUpdated();
    return;
  }

  auto *p = m_actorPort.other();
  p->node()->update();

  auto actor = p->value().Get<interop::anari::ANARIActor>();

  auto &field = actor.GetField();
  field.GetRange(&m_range);

  if (m_callback)
    m_callback(m_range);

  markUpdated();
}

Range ExtractActorFieldRangeNode::getRange() const
{
  return m_range;
}

} // namespace graph
} // namespace vtkm
