#include <chrono>

#include <gtest/gtest.h>

#include "hypertron_ros2_bridge/thread_safe_queue.hpp"

namespace hypertron_ros2_bridge {
namespace {

TEST(ThreadSafeQueue, RejectNewPreservesExistingItems) {
  ThreadSafeQueue<int> queue(2, OverflowPolicy::RejectNew);
  EXPECT_TRUE(queue.push(1));
  EXPECT_TRUE(queue.push(2));
  EXPECT_FALSE(queue.push(3));
  EXPECT_EQ(queue.try_pop(), 1);
  EXPECT_EQ(queue.try_pop(), 2);
}

TEST(ThreadSafeQueue, DropOldestKeepsFreshData) {
  ThreadSafeQueue<int> queue(2, OverflowPolicy::DropOldest);
  EXPECT_TRUE(queue.push(1));
  EXPECT_TRUE(queue.push(2));
  EXPECT_TRUE(queue.push(3));
  EXPECT_EQ(queue.try_pop(), 2);
  EXPECT_EQ(queue.try_pop(), 3);
}

TEST(ThreadSafeQueue, CloseWakesWaiterAndRejectsPush) {
  using namespace std::chrono_literals;
  ThreadSafeQueue<int> queue(1, OverflowPolicy::RejectNew);
  queue.close();
  EXPECT_FALSE(queue.push(1));
  EXPECT_FALSE(queue.wait_pop_for(1ms).has_value());
  EXPECT_TRUE(queue.closed());
}

TEST(ThreadSafeQueue, ZeroCapacityIsRejected) {
  EXPECT_THROW((ThreadSafeQueue<int>(0, OverflowPolicy::RejectNew)),
               std::invalid_argument);
}

}  // namespace
}  // namespace hypertron_ros2_bridge
