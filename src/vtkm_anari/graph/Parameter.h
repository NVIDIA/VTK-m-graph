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

// std
#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <type_traits>

namespace vtkm_anari {
namespace graph {

struct Parameter;

enum class ParameterChangeType
{
  NEW_VALUE,
  NEW_MINMAX
};

struct ParameterObserver
{
  virtual void parameterChanged(Parameter *, ParameterChangeType type) {}
};

enum class ParameterType
{
  BOUNDED_FLOAT,
  BOUNDED_INT,
  FLOAT,
  INT,
  BOOL,
  FILENAME,
  UNKNOWN
};

template <typename T>
constexpr void validParameterType()
{
  constexpr bool valid = std::is_same<T, float>::value
      || std::is_same<T, int>::value || std::is_same<T, bool>::value
      || std::is_same<T, std::string>::value;
  static_assert(
      valid, "Must use float, int, bool, or string parameter types only.");
}

struct Parameter
{
  template <typename T>
  Parameter(
      ParameterObserver *object, const char *name, ParameterType type, T value);

  const char *name() const;

  template <typename T>
  T valueAs() const;
  template <typename T>
  T minAs() const;
  template <typename T>
  T maxAs() const;

  template <typename T>
  bool isType() const;
  ParameterType type() const;

  template <typename T>
  bool setValue(T newValue);
  template <typename T>
  bool setMinMax(T newMin, T newMax, T initialValue);
  void unsetMinMax();
  bool hasMinMax() const;

  template <typename T>
  void operator=(T newValue);

  ////////////////////////////////////////

  Parameter() = delete;
  ~Parameter() = default;

  Parameter(const Parameter &) = default;
  Parameter(Parameter &&) = default;

  Parameter &operator=(const Parameter &) = default;
  Parameter &operator=(Parameter &&) = default;

 private:
  void notifyObserver(ParameterChangeType type);

  using ParameterStorage = std::array<unsigned char, sizeof(float)>;

  std::string m_stringValue; // use this if its a string
  ParameterStorage m_value;
  ParameterStorage m_min;
  ParameterStorage m_max;
  bool m_hasMinMax{false};

  ParameterObserver *m_observer{nullptr};
  std::string m_name;
  ParameterType m_type{ParameterType::UNKNOWN};
};

// Inlined definitions ////////////////////////////////////////////////////////

template <typename T>
inline Parameter::Parameter(
    ParameterObserver *object, const char *name, ParameterType type, T value)
    : m_observer(object), m_name(name), m_type(type)
{
  std::fill(m_value.begin(), m_value.end(), 0);
  unsetMinMax();
  if (!setValue(value))
    notifyObserver(ParameterChangeType::NEW_VALUE);
}

template <typename T>
inline T Parameter::valueAs() const
{
  validParameterType<T>();
  T retval;
  std::memcpy(&retval, m_value.data(), sizeof(T));
  return retval;
}

template <>
inline std::string Parameter::valueAs() const
{
  return m_stringValue;
}

template <typename T>
inline T Parameter::minAs() const
{
  validParameterType<T>();
  T retval;
  std::memcpy(&retval, m_min.data(), sizeof(T));
  return retval;
}

template <typename T>
inline T Parameter::maxAs() const
{
  validParameterType<T>();
  T retval;
  std::memcpy(&retval, m_max.data(), sizeof(T));
  return retval;
}

template <typename T>
inline bool Parameter::isType() const
{
  return (std::is_same<T, bool>::value && type() == ParameterType::BOOL)
      || (std::is_same<T, int>::value
          && (type() == ParameterType::INT
              || type() == ParameterType::BOUNDED_INT))
      || (std::is_same<T, std::string>::value
          && type() == ParameterType::FILENAME)
      || (std::is_same<T, float>::value
          && (type() == ParameterType::BOUNDED_FLOAT
              || type() == ParameterType::FLOAT));
}

template <typename T>
inline bool Parameter::setValue(T newValue)
{
  validParameterType<T>();
  const auto v = valueAs<T>();
  if (v != newValue) {
    std::memcpy(m_value.data(), &newValue, sizeof(T));
    notifyObserver(ParameterChangeType::NEW_VALUE);
    return true;
  }
  return false;
}

template <>
inline bool Parameter::setValue(std::string newValue)
{
  if (m_stringValue != newValue) {
    m_stringValue = newValue;
    notifyObserver(ParameterChangeType::NEW_VALUE);
    return true;
  }
  return false;
}

template <typename T>
inline bool Parameter::setMinMax(T newMin, T newMax, T initialValue)
{
  validParameterType<T>();
  const auto oldMin = minAs<T>();
  const auto oldMax = maxAs<T>();
  const bool notify = !hasMinMax() || oldMin != newMin || oldMax != newMax;
  std::memcpy(m_min.data(), &newMin, sizeof(T));
  std::memcpy(m_max.data(), &newMax, sizeof(T));
  if (!hasMinMax()) {
    setValue(initialValue);
    m_hasMinMax = true;
  }

  if (notify)
    notifyObserver(ParameterChangeType::NEW_MINMAX);

  return notify;
}

template <typename T>
inline void Parameter::operator=(T newValue)
{
  setValue(newValue);
}

} // namespace graph
} // namespace vtkm_anari
