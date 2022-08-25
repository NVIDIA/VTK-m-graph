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

struct VTKM_ANARI_EXPORT UtilityNode : public Node
{
  UtilityNode(bool primary = false);
  ~UtilityNode() override = default;

  NodeType type() const override;
  bool isValid() const override;
};

// Concrete node types ////////////////////////////////////////////////////////

struct VTKM_ANARI_EXPORT DataSetToComponentsNode : public UtilityNode
{
  DataSetToComponentsNode();
  ~DataSetToComponentsNode();
  const char *kind() const override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  void update() override;

 private:
  InPort m_datasetInPort{PortType::DATASET, "dataset", this};
  std::vector<OutPort> m_outPorts;
};

struct VTKM_ANARI_EXPORT ComponentsToDataSetNode : public UtilityNode
{
  ComponentsToDataSetNode();
  ~ComponentsToDataSetNode();
  const char *kind() const override;

  size_t numInput() const override;
  InPort *inputBegin() override;

  size_t numOutput() const override;
  OutPort *outputBegin() override;

  void update() override;

  std::vector<InPort> m_inPorts;
  OutPort m_datasetOutPort{PortType::DATASET, "dataset", this};
};

} // namespace graph
} // namespace vtkm_anari