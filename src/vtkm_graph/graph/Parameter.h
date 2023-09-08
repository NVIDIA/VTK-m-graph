// Copyright 2023 NVIDIA Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

// std
#include <algorithm>
#include <array>
#include <cstring>
#include <string>
#include <type_traits>

namespace vtkm {
namespace graph {

template <typename T>
constexpr void validParameterType()
{
  constexpr bool valid = std::is_same<T, float>::value
      || std::is_same<T, int>::value || std::is_same<T, bool>::value
      || std::is_same<T, std::string>::value;
  static_assert(
      valid, "Must use float, int, bool, or string parameter types only.");
}

using ParameterBuffer = std::array<unsigned char, sizeof(float)>;
struct ParameterRawValue
{
  ParameterRawValue()
  {
    std::fill(buf.begin(), buf.end(), 0);
  }

  template <typename T>
  ParameterRawValue(T v) : ParameterRawValue()
  {
    validParameterType<T>();
    std::memcpy(buf.data(), &v, sizeof(T));
  }

  ParameterRawValue(std::string v) : ParameterRawValue()
  {
    str = v;
  }

  ParameterBuffer buf;
  std::string str;
};

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
  bool setRawValue(ParameterRawValue &&v);

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

  std::string m_stringValue; // use this if its a string
  ParameterBuffer m_value;
  ParameterBuffer m_min;
  ParameterBuffer m_max;
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
  return setRawValue(newValue);
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
} // namespace vtkm
