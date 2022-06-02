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

#include "../FilterNode.h"
// vtk-m
#include <vtkm/filter/CleanGrid.h>

namespace vtkm_anari {
namespace graph {

CleanGridNode::CleanGridNode()
{
  addParameter({this, "compactPointFields", ParameterType::BOOL, true});
  addParameter({this, "mergePoints", ParameterType::BOOL, true});
  addParameter({this, "fastMerge", ParameterType::BOOL, false});
  addParameter({this, "removeDegenerateCells", ParameterType::BOOL, true});
  addParameter({this, "toleranceIsAbsolute", ParameterType::BOOL, true});
  addParameter({this, "tolerance", ParameterType::FLOAT, 0.f});
}

const char *CleanGridNode::kind() const
{
  return "CleanGrid";
}

void CleanGridNode::parameterChanged(Parameter *p, ParameterChangeType type)
{
  if (type == ParameterChangeType::NEW_VALUE) {
    if (!std::strcmp(p->name(), "compactPointFields"))
      m_compactPointFields = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "mergePoints"))
      m_mergePoints = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "fastMerge"))
      m_fastMerge = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "removeDegenerateCells"))
      m_removeDegenerateCells = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "toleranceIsAbsolute"))
      m_toleranceIsAbsolute = p->valueAs<bool>();
    if (!std::strcmp(p->name(), "tolerance"))
      m_tolerance = p->valueAs<float>();
    notifyObserver();
  }
}

vtkm::cont::DataSet CleanGridNode::execute(vtkm::cont::DataSet ds)
{
  vtkm::filter::CleanGrid filter;
  filter.SetCompactPointFields(m_compactPointFields);
  filter.SetMergePoints(m_mergePoints);
  filter.SetFastMerge(m_fastMerge);
  filter.SetRemoveDegenerateCells(m_removeDegenerateCells);
  filter.SetToleranceIsAbsolute(m_toleranceIsAbsolute);
  filter.SetTolerance(m_tolerance);
  return filter.Execute(ds);
}

} // namespace graph
} // namespace vtkm_anari
