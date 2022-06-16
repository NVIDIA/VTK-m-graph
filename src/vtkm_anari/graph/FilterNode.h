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

namespace vtkm_anari {
namespace graph {

struct VTKM_ANARI_EXPORT FilterNode : public Node
{
  FilterNode() = default;
  ~FilterNode() override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  virtual InPort *datasetInput();
  OutPort *datasetOutput();

  NodeType type() const override;
  bool isValid() const override;

  void update() override;
  vtkm::cont::DataSet dataset();

 protected:
  vtkm::cont::DataSet getDataSetFromPort(InPort *p);

 private:
  virtual vtkm::cont::DataSet execute() = 0;

  vtkm::cont::DataSet m_dataset;
  OutPort m_datasetOutPort{PortType::DATASET, "dataset", this};
  InPort m_datasetInPort{PortType::DATASET, "dataset", this};
};

using FilterNodePtr = std::unique_ptr<FilterNode>;

// Concrete node types ////////////////////////////////////////////////////////

struct VTKM_ANARI_EXPORT CellAverageNode : public FilterNode
{
  CellAverageNode() = default;
  const char *kind() const override;

 private:
  vtkm::cont::DataSet execute() override;
};

struct VTKM_ANARI_EXPORT CleanGridNode : public FilterNode
{
  CleanGridNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;

  bool m_compactPointFields{true};
  bool m_mergePoints{true};
  bool m_fastMerge{false};
  bool m_removeDegenerateCells{true};
};

struct VTKM_ANARI_EXPORT ContourNode : public FilterNode
{
  ContourNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;

  float m_currentIsovalue{0.f};
};

struct VTKM_ANARI_EXPORT GradientNode : public FilterNode
{
  GradientNode() = default;
  const char *kind() const override;

 private:
  vtkm::cont::DataSet execute() override;
};

struct VTKM_ANARI_EXPORT PointAverageNode : public FilterNode
{
  PointAverageNode() = default;
  const char *kind() const override;

 private:
  vtkm::cont::DataSet execute() override;
};

struct VTKM_ANARI_EXPORT SliceNode : public FilterNode
{
  SliceNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;

  vtkm::Vec3f m_azelpos{0.f, 0.f, 0.5f};
};

struct VTKM_ANARI_EXPORT StreamlineNode : public FilterNode
{
  StreamlineNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

  InPort *inputBegin() override;
  size_t numInput() const override;

  InPort *datasetInput() override;
  InPort *streamlineCoordsInput();

 private:
  vtkm::cont::DataSet execute() override;

  InPort m_mainDataInPort{PortType::DATASET, "dataset", this};
  InPort m_coordsInPort{PortType::DATASET, "streamline coords", this};

  int m_steps{100};
  float m_stepSize{0.f};
};

struct VTKM_ANARI_EXPORT SurfaceNormalsNode : public FilterNode
{
  SurfaceNormalsNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;

  bool m_oriented{true};
  bool m_flip{true};
};

struct VTKM_ANARI_EXPORT TubeNode : public FilterNode
{
  TubeNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;

  int m_sides{7};
  float m_radius{0.f};
  bool m_cap{false};
};

struct VTKM_ANARI_EXPORT VectorMagnitudeNode : public FilterNode
{
  VectorMagnitudeNode() = default;
  const char *kind() const override;

 private:
  vtkm::cont::DataSet execute() override;
};

struct VTKM_ANARI_EXPORT VertexClusteringNode : public FilterNode
{
  VertexClusteringNode();
  const char *kind() const override;
  void parameterChanged(Parameter *p, ParameterChangeType type) override;

 private:
  vtkm::cont::DataSet execute() override;
};

} // namespace graph
} // namespace vtkm_anari
