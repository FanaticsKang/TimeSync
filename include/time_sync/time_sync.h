#pragma once
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <type_traits>

#include "circle_queue.h"

struct NullType {};

template <class M0, class M1, class M2 = NullType, class M3 = NullType>
class TimeSync {
  typedef std::pair<double, M0> M0Pair;
  typedef std::shared_ptr<M0Pair> M0PairPtr;
  typedef CircleQueue<M0Pair> M0Circle;
  typedef std::shared_ptr<CircleQueue<M0Pair>> M0CirclePtr;

  typedef std::pair<double, M1> M1Pair;
  typedef std::shared_ptr<M1Pair> M1PairPtr;
  typedef CircleQueue<M1Pair> M1Circle;
  typedef std::shared_ptr<CircleQueue<M1Pair>> M1CirclePtr;

  typedef std::pair<double, M2> M2Pair;
  typedef std::shared_ptr<M2Pair> M2PairPtr;
  typedef CircleQueue<M2Pair> M2Circle;
  typedef std::shared_ptr<CircleQueue<M2Pair>> M2CirclePtr;

  typedef std::pair<double, M3> M3Pair;
  typedef std::shared_ptr<M3Pair> M3PairPtr;
  typedef CircleQueue<M3Pair> M3Circle;
  typedef std::shared_ptr<CircleQueue<M3Pair>> M3CirclePtr;

 public:
  TimeSync(const size_t queue_size) : class_size_(2) {
    data_queues_.resize(4);
    // 注意目前的last_time
    data_queues_[0] = std::make_shared<CircleQueue<M0Pair>>(queue_size);
    data_queues_[1] = std::make_shared<CircleQueue<M1Pair>>(queue_size);
    if (!std::is_same<M2, NullType>::value) {
      data_queues_[2] = std::make_shared<CircleQueue<M2Pair>>(queue_size);
      ++class_size_;
    }
    if (!std::is_same<M3, NullType>::value) {
      data_queues_[3] = std::make_shared<CircleQueue<M3Pair>>(queue_size);
      ++class_size_;
      NullType test;
    }
    std::cout << "TimeSync class size: " << class_size_ << std::endl;
    last_times_.resize(class_size_, 0);
  };

  template <class T>
  void PushMsg(const T &data, const int index, const double time) {
    std::shared_ptr<std::pair<double, T>> m0_ptr =
        std::make_shared<std::pair<double, T>>(time, data);
    PushMsg<T>(m0_ptr, index);
  }

  template <class T>
  void PushMsg(std::shared_ptr<std::pair<double, T>> data, const int index) {
    std::lock_guard<std::mutex> guard(lock_);
    std::shared_ptr<CircleQueue<std::pair<double, T>>> queue =
        std::static_pointer_cast<CircleQueue<std::pair<double, T>>>(
            data_queues_[index]);
    queue->Insert(data);
    last_times_[index] = data->first;
    TryToProcess(data->first);
  }

 private:
  void TryToProcess(const double time) {
    if (fabs(MinTime() - time) > 1e-7) {
      return;
    }
    M0PairPtr m0 = nullptr;
    M1PairPtr m1 = nullptr;
    M2PairPtr m2 = nullptr;
    M3PairPtr m3 = nullptr;
    for (size_t i = 0; i < class_size_; ++i) {
      if (i == 0) {
        m0 = FindValue<M0Pair>(time, data_queues_[0]);
      }
      if (i == 1) {
        m1 = FindValue<M1Pair>(time, data_queues_[1]);
      }
      if (i == 2) {
        m2 = FindValue<M2Pair>(time, data_queues_[2]);
      }
      if (i == 3) {
        m3 = FindValue<M3Pair>(time, data_queues_[3]);
      }
    }
    const bool m0_ok = m0 != nullptr;
    const bool m1_ok = m1 != nullptr;
    const bool m2_ok = m2 != nullptr;
    const bool m3_ok = m3 != nullptr;

    if (m0_ok && m1_ok && class_size_ == 2) {
      function_call_back_(m0->second, m1->second, M2(), M3());
    }

    if (m0_ok && m1_ok && m2_ok && class_size_ == 3) {
      function_call_back_(m0->second, m1->second, m2->second, M3());
    }

    if (m0_ok && m1_ok && m2_ok && m3_ok && class_size_ == 4) {
      function_call_back_(m0->second, m1->second, m2->second, m3->second);
    }
  }

  template <class T>
  std::shared_ptr<T> FindValue(const double time, CircleBasePtr base) {
    std::shared_ptr<CircleQueue<T>> value =
        std::static_pointer_cast<CircleQueue<T>>(base);
    const size_t queue_size = value->QueueSize();
    for (size_t i = 0; i < queue_size; ++i) {
      std::shared_ptr<T> test = value->GetQueueElement(i);
      if (test != nullptr && fabs(time - test->first) < 1e-7) {
        return test;
      }
    }
    return nullptr;
  }

  double MinTime() {
    return *std::min_element(last_times_.begin(), last_times_.end());
  }

 public:
  std::function<void(const M0 &, const M1 &, const M2 &, const M3 &)>
      function_call_back_;

 private:
  std::mutex lock_;
  std::vector<CircleBasePtr> data_queues_;
  std::vector<double> last_times_;
  size_t class_size_;
};
