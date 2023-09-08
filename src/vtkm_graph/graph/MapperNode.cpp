// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "MapperNode.h"
#include "ActorNode.h"

namespace vtkm {
namespace graph {

MapperNode::MapperNode() : Node(true)
{
  addParameter({this, "visible", ParameterType::BOOL, true});
}

MapperNode::~MapperNode()
{
  m_actorPort.disconnect();
  m_scene->RemoveMapper(m_scene->GetMapperIndexByName(name()));
}

InPort *MapperNode::inputBegin()
{
  return &m_actorPort;
}

size_t MapperNode::numInput() const
{
  return 1;
}

NodeType MapperNode::type() const
{
  return NodeType::MAPPER;
}

bool MapperNode::isValid() const
{
  return m_actorPort.isConnected();
}

void MapperNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "visible")) {
      m_visible = p->valueAs<bool>();
      if (m_scene) {
        m_scene->SetMapperVisible(
            m_scene->GetMapperIndexByName(name()), m_visible);
      }
    }
  }
  markChanged();
}

bool MapperNode::isVisible() const
{
  return m_visible;
}

bool MapperNode::isMapperEmpty() const
{
  return m_mapper->GroupIsEmpty();
}

interop::anari::ANARIMapper *MapperNode::getMapper() const
{
  return m_mapper;
}

void MapperNode::update()
{
  if (!m_mapper || !isVisible() || !needsUpdate())
    return;

  if (!m_actorPort.isConnected()) {
    m_mapper->SetActor({});
    markUpdated();
    return;
  }

  auto *p = m_actorPort.other();
  p->node()->update();
  m_mapper->SetActor(p->value().Get<interop::anari::ANARIActor>());
  markUpdated();
}

} // namespace graph
} // namespace vtkm
