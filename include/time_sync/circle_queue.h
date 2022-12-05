#pragma once

#include <iostream>
#include <memory>
#include <vector>

class CircleBase {
};

template <typename T>
class CircleQueue : public CircleBase {
 public:
  explicit CircleQueue(const size_t queue_size) : size_(queue_size) {
    if (queue_size == 0) {
    }

    queue_.resize(size_, nullptr);
    index_ = 0;
  }

  CircleQueue() = delete;

  void Insert(const std::shared_ptr<T>& elem) {
    queue_[index_] = elem;
    ++index_;
    index_ %= size_;
  }

  int QueueSize() const { return size_; }

  int NextIndex() const { return index_; }

  int CurrentIndex() const {
    if (index_ == 0) {
      return size_ - 1;
    }
    return index_;
  }

  const std::shared_ptr<T> GetQueueElement(const size_t index) const {
    if (index < 0 || index >= size_) {
      return nullptr;
    }
    return queue_[index];
  }

 private:
  int index_ = 0;
  const size_t size_;
  std::vector<std::shared_ptr<T>> queue_;
};

typedef std::shared_ptr<CircleBase> CircleBasePtr;