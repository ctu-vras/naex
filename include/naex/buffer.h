#pragma once

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace naex {

template <typename T> void noop(T *) {}

/// Shared, reference-counted array with an optional external owner.
template <typename T> class Buffer {
public:
  Buffer() : begin_(), size_(0) {}

  Buffer(const Buffer<T> &other) : begin_(other.begin_), size_(other.size_) {}

  Buffer<T> &operator=(const Buffer<T> &other) = default;

  explicit Buffer(std::shared_ptr<T> other) : begin_(other), size_(1) {}

  explicit Buffer(size_t size)
      : begin_(new T[size], std::default_delete<T[]>()), size_(size) {}

  Buffer(T *begin, size_t size) : begin_(begin, noop<T>), size_(size) {
    assert(begin != nullptr);
    assert(size > 0);
  }

  Buffer(std::shared_ptr<T> begin, size_t size) : begin_(begin), size_(size) {
    assert(begin.get() != nullptr);
    assert(size > 0);
  }

  Buffer(T *begin, T *end)
      : begin_(begin, noop<T>), size_(static_cast<size_t>(end - begin)) {
    assert(begin != nullptr);
    assert(end != nullptr);
    assert(begin <= end);
  }

  Buffer(T *begin, T *end, std::function<void(T *)> deleter)
      : begin_(begin, deleter), size_(static_cast<size_t>(end - begin)) {
    assert(begin != nullptr);
    assert(end != nullptr);
    assert(begin <= end);
  }

  size_t size() const { return size_; }

  bool empty() const { return size() == 0; }

  void resize(size_t size) {
    begin_ = std::shared_ptr<T>(new T[size], std::default_delete<T[]>());
    size_ = size;
  }

  Buffer<T> copy(size_t size) {
    Buffer<T> out(size);
    std::copy(begin(), begin() + std::min(this->size(), size), out.begin());
    return out;
  }

  T *begin() { return begin_.get(); }

  const T *begin() const { return begin_.get(); }

  T *end() {
    if (empty()) {
      return nullptr;
    }
    return begin_.get() + size_;
  }

  const T *end() const {
    if (empty()) {
      return nullptr;
    }
    return begin_.get() + size_;
  }

  T &operator[](size_t i) {
    assert(i < size());
    return *(begin_.get() + i);
  }

  const T &operator[](size_t i) const {
    assert(i < size());
    return *(begin_.get() + i);
  }

  template <typename D> D *data() {
    return reinterpret_cast<D *>(begin_.get());
  }

  template <typename D> const D *data() const {
    return reinterpret_cast<const D *>(begin_.get());
  }

  template <typename D> explicit operator D *() { return data<D>(); }

  explicit operator std::string() const {
    if (empty()) {
      return {};
    }
    return std::string(data<char>(), size());
  }

protected:
  std::shared_ptr<T> begin_;
  size_t size_;
};

} // namespace naex
