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

#include "../ANARIActor.h"
#include "Node.h"
// vtk-m
#include <vtkm/cont/DataSet.h>

namespace vtkm_anari {
namespace graph {

struct VTKM_ANARI_EXPORT ActorNode : public Node
{
  ActorNode();
  ~ActorNode() override;

  const char *kind() const override;

  InPort *input(const char *name) override;
  OutPort *output(const char *name) override;

  NodeType type() const override;
  bool isValid() const override;

  ANARIActor makeActor(vtkm::cont::DataSet ds);
  void setFieldNames(vtkm::cont::DataSet ds);

  size_t numFields() const;
  const char *fieldName(size_t i) const;
  size_t getCurrentField() const;
  void setCurrentField(size_t i);

 private:
  InPort m_datasetPort{PortType::DATASET, "dataset", this};
  OutPort m_actorPort{PortType::ACTOR, "actor", this};
  std::vector<std::string> m_fields;
  size_t m_currentField{0};
};

using ActorNodePtr = std::unique_ptr<ActorNode>;

} // namespace graph
} // namespace vtkm_anari
