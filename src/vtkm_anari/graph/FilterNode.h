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

#include "Node.h"
// vtk-m
#include <vtkm/cont/DataSet.h>

namespace vtkm_anari::graph {

struct VTKM_ANARI_EXPORT FilterNode : public Node
{
  FilterNode() = default;
  ~FilterNode() override;

  virtual vtkm::cont::DataSet execute(vtkm::cont::DataSet) = 0;

  InPort *input(const char *name) override;
  OutPort *output(const char *name) override;

  NodeType type() const override;
  bool isValid() const override;

 private:
  InPort m_datasetInPort{PortType::DATASET, "dataset", this};
  OutPort m_datasetOutPort{PortType::DATASET, "dataset", this};
};

using FilterNodePtr = std::unique_ptr<FilterNode>;

// Concrete node types ////////////////////////////////////////////////////////

struct VTKM_ANARI_EXPORT ContourNode : public FilterNode
{
  ContourNode() = default;
  const char *kind() const override;
  vtkm::cont::DataSet execute(vtkm::cont::DataSet) override;
};

struct VTKM_ANARI_EXPORT GradientNode : public FilterNode
{
  GradientNode() = default;
  const char *kind() const override;
  vtkm::cont::DataSet execute(vtkm::cont::DataSet) override;
};

struct VTKM_ANARI_EXPORT CleanGridNode : public FilterNode
{
  CleanGridNode() = default;
  const char *kind() const override;
  vtkm::cont::DataSet execute(vtkm::cont::DataSet) override;
};

struct VTKM_ANARI_EXPORT VectorMagnitudeNode : public FilterNode
{
  VectorMagnitudeNode() = default;
  const char *kind() const override;
  vtkm::cont::DataSet execute(vtkm::cont::DataSet) override;
};

struct VTKM_ANARI_EXPORT PointAverageNode : public FilterNode
{
  PointAverageNode() = default;
  const char *kind() const override;
  vtkm::cont::DataSet execute(vtkm::cont::DataSet) override;
};

struct VTKM_ANARI_EXPORT CellAverageNode : public FilterNode
{
  CellAverageNode() = default;
  const char *kind() const override;
  vtkm::cont::DataSet execute(vtkm::cont::DataSet) override;
};

} // namespace vtkm_anari::graph
