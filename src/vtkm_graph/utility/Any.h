/*
 * Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <string.h>
#include <memory>
#include <stdexcept>

namespace vtkm {

/* 'Any' implements a single item container which erases its type (can hold
 *  any value which is copyable). The value can be extracted successfully
 *  only if the correct type is queried for the held value, where an
 *  exception is thrown otherwise. Similar (but perhaps not identical to)
 *  'boost::any' or C++17's 'std::any'.
 *
 *  Example:
 *
 *      Any myAny = 1;                 // myAny is an 'int' w/ value of '1'
 *      int value = myAny.get<int>();  // get value of '1' out of myAny
 *      char bad  = myAny.get<char>(); // throws exception
 */
struct Any
{
  Any() = default;
  Any(const Any &copy);

  template <typename T>
  Any(T value);

  ~Any() = default;

  Any &operator=(const Any &rhs);

  template <typename T>
  Any &operator=(T rhs);

  template <typename T>
  T &Get();

  template <typename T>
  const T &Get() const;

  template <typename T>
  T Value() const;

  template <typename T>
  bool Is() const;

  bool Valid() const;
  void Reset();

 private:
  // Helper types //

  struct handle_base
  {
    virtual ~handle_base() = default;
    virtual handle_base *Clone() const = 0;
    virtual const std::type_info &ValueTypeID() const = 0;
    virtual void *Data() = 0;
  };

  template <typename T>
  struct handle : public handle_base
  {
    handle(T value);
    handle_base *Clone() const override;
    const std::type_info &ValueTypeID() const override;
    void *Data() override;
    T value;
  };

  // Data members //

  std::unique_ptr<handle_base> currentValue;
};

// Inlined Any definitions ////////////////////////////////////////////////////

template <typename T>
inline Any::Any(T value)
    : currentValue(new handle<typename std::remove_reference<T>::type>(
        std::forward<T>(value)))
{
  static_assert(
      std::is_copy_constructible<T>::value && std::is_copy_assignable<T>::value,
      "Any can only be constructed with copyable values!");
}

inline Any::Any(const Any &copy)
    : currentValue(copy.Valid() ? copy.currentValue->Clone() : nullptr)
{}

inline Any &Any::operator=(const Any &rhs)
{
  Any temp(rhs);
  currentValue = std::move(temp.currentValue);
  return *this;
}

template <typename T>
inline Any &Any::operator=(T rhs)
{
  static_assert(
      std::is_copy_constructible<T>::value && std::is_copy_assignable<T>::value,
      "Any can only be assigned values which are copyable!");

  currentValue = std::unique_ptr<handle_base>(
      new handle<typename std::remove_reference<T>::type>(
          std::forward<T>(rhs)));

  return *this;
}

template <typename T>
inline T &Any::Get()
{
  if (!Valid())
    throw std::runtime_error("Can't query value from an empty Any");

  if (Is<T>())
    return *(static_cast<T *>(currentValue->Data()));
  else
    throw std::runtime_error("Invalid type when querying Any");
}

template <typename T>
inline const T &Any::Get() const
{
  if (!Valid())
    throw std::runtime_error("Can't query value from an empty Any");

  if (Is<T>())
    return *(static_cast<T *>(currentValue->Data()));
  else
    throw std::runtime_error("Invalid type when querying Any");
}

template <typename T>
inline T Any::Value() const
{
  if (!Valid() || !Is<T>()) {
#if 0
    printf(
        "Any::Value --> valid: %i | Is<T>(): %i\n", int(Valid()), int(Is<T>()));
#endif
    return {};
  } else
    return Get<T>();
}

template <typename T>
inline bool Any::Is() const
{
  return Valid()
      && (strcmp(typeid(T).name(), currentValue->ValueTypeID().name()) == 0);
}

inline bool Any::Valid() const
{
  return currentValue.get() != nullptr;
}

inline void Any::Reset()
{
  return currentValue.reset();
}

template <typename T>
inline Any::handle<T>::handle(T v) : value(std::move(v))
{}

template <typename T>
inline Any::handle_base *Any::handle<T>::Clone() const
{
  return new handle<T>(value);
}

template <typename T>
inline const std::type_info &Any::handle<T>::ValueTypeID() const
{
  return typeid(T);
}

template <typename T>
inline void *Any::handle<T>::Data()
{
  return &value;
}

} // namespace vtkm
