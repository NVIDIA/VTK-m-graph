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
// std
#include <functional>

namespace vtkm_anari {
namespace graph {

struct VTKM_ANARI_EXPORT SourceNode : public Node
{
  SourceNode() = default;
  ~SourceNode() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  NodeType type() const override;
  bool isValid() const override;

  void update() override;

 private:
  virtual vtkm::cont::DataSet execute() = 0;

  OutPort m_datasetPort{PortType::DATASET, "dataset", this};
};

// Concrete node types ////////////////////////////////////////////////////////

struct VTKM_ANARI_EXPORT ABCSourceNode : public SourceNode
{
  ABCSourceNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;
};

using SourceNodeCallback = std::function<vtkm::cont::DataSet()>;

struct VTKM_ANARI_EXPORT CallbackSourceNode : public SourceNode
{
  CallbackSourceNode() = default;
  CallbackSourceNode(SourceNodeCallback cb);
  const char *kind() const override;

  void setCallback(SourceNodeCallback cb);

  void markChanged(); // Application signal to refresh dataset via callback

 private:
  vtkm::cont::DataSet execute() override;

  SourceNodeCallback m_callback = []() -> vtkm::cont::DataSet { return {}; };
};

struct VTKM_ANARI_EXPORT RandomPointsSourceNode : public SourceNode
{
  RandomPointsSourceNode() = default;
  const char *kind() const override;

 private:
  vtkm::cont::DataSet execute() override;
};

struct VTKM_ANARI_EXPORT TangleSourceNode : public SourceNode
{
  TangleSourceNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;
};

struct VTKM_ANARI_EXPORT VTKFileReaderNode : public SourceNode
{
  VTKFileReaderNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;
};

} // namespace graph
} // namespace vtkm_anari
