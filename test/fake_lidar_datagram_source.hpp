#pragma once

#include <array>
#include <condition_variable>
#include <cstdint>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include "hypertron_ros2_bridge/lidar_stream.hpp"

namespace hypertron_ros2_bridge {
namespace test {

// Programmable ILidarDatagramSource for receiver tests. Datagrams pushed via
// push() are delivered in FIFO order on the next receive(); a forced timeout
// can be set with set_blocked(true) to exercise the receiver's no-data path.
// receive() returns false on timeout without consuming anything, mirroring
// the real UDP source.
//
// The queue is a fixed-size ring buffer of preallocated slots and every
// access (including the condition-variable notify) happens while holding the
// one mutex, so ThreadSanitizer sees the producer/receiver handoff as fully
// ordered and reports no races in this harness.
class FakeLidarDatagramSource final : public ILidarDatagramSource {
 public:
  static constexpr std::size_t kCapacity = 64;

  FakeLidarDatagramSource() = default;
  ~FakeLidarDatagramSource() override = default;
  FakeLidarDatagramSource(const FakeLidarDatagramSource&) = delete;
  FakeLidarDatagramSource& operator=(const FakeLidarDatagramSource&) = delete;

  // Enqueues one datagram for the next receive() and wakes any waiting
  // receive call. No-ops (and returns false) when the ring is full.
  bool push(std::vector<std::uint8_t> datagram) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (count_ >= kCapacity) {
        return false;
      }
      slots_[write_].swap(datagram);
      write_ = (write_ + 1) % kCapacity;
      ++count_;
      blocked_ = false;
      cv_.notify_all();  // notify while holding the mutex so TSAN orders it
    }
    return true;
  }

  // When blocked, receive() waits for the full timeout and returns false
  // (no data consumed), regardless of any queued datagrams.
  void set_blocked(bool blocked) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      blocked_ = blocked;
      cv_.notify_all();
    }
  }

  std::size_t datagram_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return count_;
  }

  // Bounded wait: returns true when at least one datagram is queued within
  // `timeout`.
  bool wait_until_queued(std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_until(lock, std::chrono::steady_clock::now() + timeout,
                          [this] { return count_ > 0; });
  }

  bool receive(std::vector<std::uint8_t>& out,
               std::chrono::milliseconds timeout) override {
    std::unique_lock<std::mutex> lock(mutex_);
    auto deadline = std::chrono::steady_clock::now() + timeout;
    cv_.wait_until(lock, deadline,
                   [this] { return count_ > 0 || blocked_; });
    if (blocked_ || count_ == 0) {
      ++timeouts_;
      return false;
    }
    // Copy out and consume the slot while still holding the mutex so the
    // datagram bytes written by push() are ordered before this read.
    std::vector<std::uint8_t>& slot = slots_[read_];
    out.insert(out.end(), slot.begin(), slot.end());
    slot.clear();
    read_ = (read_ + 1) % kCapacity;
    --count_;
    ++received_;
    return true;
  }

  std::uint64_t received() const { return received_; }
  std::uint64_t timeouts() const { return timeouts_; }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  // Fixed ring buffer: slots[read_ .. write_-1 mod kCapacity] hold datagrams.
  std::array<std::vector<std::uint8_t>, kCapacity> slots_;
  std::size_t read_{0};
  std::size_t write_{0};
  std::size_t count_{0};
  bool blocked_{false};
  std::uint64_t received_{0};
  std::uint64_t timeouts_{0};
};

}  // namespace test
}  // namespace hypertron_ros2_bridge
