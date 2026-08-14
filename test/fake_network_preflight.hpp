#pragma once

#include <vector>

#include "hypertron_ros2_bridge/network_preflight.hpp"

namespace hypertron_ros2_bridge {
namespace test {

// Programmable INetworkPreflight for runtime and boundary tests. Records
// every expectation it is asked to check and returns a fixed decision.
// Never touches the host network configuration.
class FakeNetworkPreflight final : public INetworkPreflight {
 public:
  NetworkDecision next_decision;
  std::vector<NetworkExpectation> expected_seen;

  NetworkDecision check(const NetworkExpectation& expected) override {
    expected_seen.push_back(expected);
    return next_decision;
  }
};

}  // namespace test
}  // namespace hypertron_ros2_bridge
