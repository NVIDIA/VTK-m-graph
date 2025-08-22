// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../UtilityNode.h"
// viskores
#include <viskores/io/VTKDataSetWriter.h>

namespace viskores {
namespace graph {

VTKFileWriterNode::VTKFileWriterNode() : UtilityNode(true)
{
  addParameter({this, "filename", ParameterType::FILENAME, std::string()});
}

const char *VTKFileWriterNode::kind() const
{
  return "VTKFileWriter";
}

InPort *VTKFileWriterNode::inputBegin()
{
  return &m_datasetInPort;
}

size_t VTKFileWriterNode::numInput() const
{
  return 1;
}

void VTKFileWriterNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_MINMAX)
    return;

  markChanged();
}

void VTKFileWriterNode::update()
{
  if (!needsUpdate())
    return;
  auto filename = parameter("filename")->valueAs<std::string>();
  if (filename.empty())
    return;
  auto ds = getDataSetFromPort(&m_datasetInPort);
  io::VTKDataSetWriter(filename).WriteDataSet(ds);
}

} // namespace graph
} // namespace viskores