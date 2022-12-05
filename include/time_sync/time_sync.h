#pragma once
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>

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
  TimeSync(const size_t class_size, const size_t queue_size)
      : class_size_(class_size) {
    data_queues_.resize(4);
    // 注意目前的last_time
    last_times_.resize(class_size, 0);
    data_queues_[0] = std::make_shared<CircleQueue<M0Pair>>(queue_size);
    data_queues_[1] = std::make_shared<CircleQueue<M1Pair>>(queue_size);
    data_queues_[2] = std::make_shared<CircleQueue<M2Pair>>(queue_size);
    data_queues_[3] = std::make_shared<CircleQueue<M3Pair>>(queue_size);
  };

  void PushMsg0(const M0 &data, const double time) {
    M0PairPtr m0_ptr = std::make_shared<M0Pair>(time, data);
    PushMsg0(m0_ptr);
  }

  void PushMsg0(M0PairPtr data) {
    std::lock_guard<std::mutex> guard(lock_);
    M0CirclePtr queue = std::static_pointer_cast<M0Circle>(data_queues_[0]);
    std::cout << "Msg0: " << std::setprecision(13) << data->first << std::endl;
    queue->Insert(data);
    last_times_[0] = data->first;
    TryToProcess(data->first);
  }

  void PushMsg1(const M1 &data, const double time) {
    M1PairPtr m1_ptr = std::make_shared<M1Pair>(time, data);
    PushMsg1(m1_ptr);
  }

  void PushMsg1(M1PairPtr data) {
    std::lock_guard<std::mutex> guard(lock_);
    M1CirclePtr queue = std::static_pointer_cast<M1Circle>(data_queues_[1]);
    queue->Insert(data);
    std::cout << "Msg1: " << std::setprecision(13) << data->first << std::endl;
    last_times_[1] = data->first;
    TryToProcess(data->first);
  }

  void PushMsg2(const M2 &data, const double time) {
    M2PairPtr m2_ptr = std::make_shared<M2Pair>(time, data);
    PushMsg2(m2_ptr);
  }

  void PushMsg2(M2PairPtr data) {
    std::lock_guard<std::mutex> guard(lock_);
    M2CirclePtr queue = std::static_pointer_cast<M2Circle>(data_queues_[2]);
    queue->Insert(data);
    std::cout << "Msg2: " << std::setprecision(13) << data->first << std::endl;
    last_times_[2] = data->first;
    TryToProcess(data->first);
  }

  void PushMsg3(const M3 &data, const double time) {
    M3PairPtr m3_ptr = std::make_shared<M3Pair>(time, data);
    PushMsg3(m3_ptr);
  }

  void PushMsg3(M3PairPtr data) {
    std::lock_guard<std::mutex> guard(lock_);
    M3CirclePtr queue = std::static_pointer_cast<M3Circle>(data_queues_[3]);
    queue->Insert(data);
    std::cout << "Msg3: " << std::setprecision(13) << data->first << std::endl;
    last_times_[3] = data->first;
    TryToProcess(data->first);
  }

 private:
  void TryToProcess(const double time) {
    if (fabs(MinTime() - time) > 1e-7) {
      return;
    }
    std::cout << "Min time: " << std::setprecision(13) << time << std::endl;
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
    if (m0 != nullptr && m1 != nullptr && m2 != nullptr) {
      std::cout << "M0 time: " << m0->first << std::endl;
      std::cout << "M1 time: " << m1->first << std::endl;
      std::cout << "M2 time: " << m2->first << std::endl;
    }
  }

  template <class T>
  std::shared_ptr<T> FindValue(const double time, CircleBasePtr base) {
    std::shared_ptr<CircleQueue<T>> value =
        std::static_pointer_cast<CircleQueue<T>>(base);
    const size_t queue_size = value->QueueSize();
    // std::cout << "\033[031m"
    //           << "queue size: " << queue_size << "\033[0m" << std::endl;
    for (size_t i = 0; i < queue_size; ++i) {
      std::shared_ptr<T> test = value->GetQueueElement(i);
      if (test != nullptr && fabs(time - test->first) < 1e-7) {
        return test;
      }
    }
    return nullptr;
  }

  double MinTime() {
    std::cout << "last time size: " << last_times_.size() << std::endl;
    return *std::min_element(last_times_.begin(), last_times_.end());
  }

 private:
  std::mutex lock_;
  std::vector<CircleBasePtr> data_queues_;
  std::vector<double> last_times_;
  const size_t class_size_;
};
