// Copyright 2023-2024 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../SourceNode.h"
// viskores
#include <viskores/io/VTKDataSetReader.h>

namespace viskores {
namespace graph {

VTKFileReaderNode::VTKFileReaderNode()
{
  addParameter({this, "filename", ParameterType::FILENAME, std::string()});
}

const char *VTKFileReaderNode::kind() const
{
  return "VTKFileReader";
}

void VTKFileReaderNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_MINMAX)
    return;

  markChanged();
}

cont::DataSet VTKFileReaderNode::execute()
{
  auto filename = parameter("filename")->valueAs<std::string>();
  return io::VTKDataSetReader(filename).ReadDataSet();
}

} // namespace graph
} // namespace viskores