#pragma once

#include "naex/types.h"

namespace naex {

/**
 * Primitive type value iterator.
 *
 * @tparam V Value type
 */
template <typename V> class ValueIterator {
public:
  typedef std::random_access_iterator_tag iterator_category;
  typedef V value_type;
  typedef std::ptrdiff_t difference_type;
  typedef V *pointer;
  typedef V reference;
  ValueIterator() : value_{} {}
  ValueIterator(const V &val) : value_(val) {}
  ValueIterator &operator++() {
    value_++;
    return *this;
  }
  ValueIterator operator++(int) {
    ValueIterator tmp = *this;
    ++value_;
    return tmp;
  }

  ValueIterator &operator--() {
    value_--;
    return *this;
  }
  bool operator!=(const ValueIterator<V> &other) const {
    return value_ != other.value_;
  }
  V &operator*() { return value_; }
  const V &operator*() const { return value_; }

  bool operator==(const ValueIterator<V> &other) const {
    return value_ == other.value_;
  }

private:
  V value_;
};

typedef ValueIterator<Vertex> VertexIter;
typedef ValueIterator<Edge> EdgeIter;

} // namespace naex
