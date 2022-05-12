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

#include "SourceNode.h"
// vtk-m
#include <vtkm/cont/DataSetBuilderExplicit.h>
#include <vtkm/source/Tangle.h>
// std
#include <random>

namespace vtkm_anari::graph {

SourceNode::~SourceNode()
{
  m_datasetPort.disconnectAllDownstreamPorts();
}

OutPort *SourceNode::output(const char *name)
{
  if (!std::strcmp(name, m_datasetPort.name()))
    return &m_datasetPort;
  return nullptr;
}

NodeType SourceNode::type() const
{
  return NodeType::SOURCE;
}

bool SourceNode::isValid() const
{
  return true;
}

// TangleSourceNode //

const char *TangleSourceNode::kind() const
{
  return "TangleSource";
}

vtkm::cont::DataSet TangleSourceNode::dataset()
{
  return vtkm::source::Tangle(vtkm::Id3{64}).Execute();
}

// RandomPointsSourceNode //

const char *RandomPointsSourceNode::kind() const
{
  return "RandomPointsSource";
}

vtkm::cont::DataSet RandomPointsSourceNode::dataset()
{
  constexpr int numSpheres = 1e4;

  std::mt19937 rng;
  rng.seed(0);
  std::normal_distribution<float> vert_dist(0.f, 4.f);

  vtkm::cont::DataSetBuilderExplicitIterative builder;

  for (int i = 0; i < numSpheres; i++) {
    builder.AddPoint(vert_dist(rng), vert_dist(rng), vert_dist(rng));
    builder.AddCell(vtkm::CELL_SHAPE_VERTEX);
    builder.AddCellPoint(i);
  }

  return builder.Create();
}

} // namespace vtkm_anari::graph