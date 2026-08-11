#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace hypertron_ros2_bridge {

enum class OverflowPolicy { RejectNew, DropOldest };

// A bounded multi-producer/multi-consumer queue. Control queues use RejectNew
// so acknowledgements are never lost; sensor/video queues use DropOldest so
// delayed data cannot build unbounded latency.
template <typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue(std::size_t capacity, OverflowPolicy policy)
      : capacity_(capacity), policy_(policy) {
    if (capacity_ == 0U) {
      throw std::invalid_argument("queue capacity must be greater than zero");
    }
  }

  ThreadSafeQueue(const ThreadSafeQueue&) = delete;
  ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

  bool push(T value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (closed_) {
        return false;
      }
      if (items_.size() == capacity_) {
        if (policy_ == OverflowPolicy::RejectNew) {
          return false;
        }
        items_.pop_front();
      }
      items_.push_back(std::move(value));
    }
    cv_.notify_one();
    return true;
  }

  std::optional<T> try_pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    return pop_locked();
  }

  std::optional<T> wait_pop_for(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, timeout, [this] { return closed_ || !items_.empty(); });
    return pop_locked();
  }

  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  bool closed() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return closed_;
  }

  std::size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return items_.size();
  }

 private:
  std::optional<T> pop_locked() {
    if (items_.empty()) {
      return std::nullopt;
    }
    T value = std::move(items_.front());
    items_.pop_front();
    return value;
  }

  const std::size_t capacity_;
  const OverflowPolicy policy_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<T> items_;
  bool closed_{false};
};

}  // namespace hypertron_ros2_bridge
