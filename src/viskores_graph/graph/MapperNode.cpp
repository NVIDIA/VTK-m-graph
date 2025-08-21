// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "MapperNode.h"
#include "ActorNode.h"

namespace viskores {
namespace graph {

MapperNode::MapperNode() : Node(true)
{
  addParameter({this, "visible", ParameterType::BOOL, true});
}

MapperNode::~MapperNode()
{
  m_actorPort.disconnect();
  m_scene->RemoveMapper(m_scene->GetMapperIndexByName(uniqueName()));
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
            m_scene->GetMapperIndexByName(uniqueName()), m_visible);
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

  updateUpstreamNodes();

  if (!m_actorPort.isConnected()) {
    m_mapper->SetActor({});
    markUpdated();
    return;
  }

  auto *p = m_actorPort.other();
  m_mapper->SetActor(p->value().Get<interop::anari::ANARIActor>());
  markUpdated();
}

void MapperNode::updateUpstreamNodes()
{
  if (m_actorPort.isConnected())
    m_actorPort.other()->node()->update();
}

} // namespace graph
} // namespace viskores
