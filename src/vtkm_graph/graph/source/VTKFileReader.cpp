// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../SourceNode.h"
// vtk-m
#include <vtkm/io/VTKDataSetReader.h>

namespace vtkm {
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
} // namespace vtkm