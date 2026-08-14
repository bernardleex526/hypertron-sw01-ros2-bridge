#include <chrono>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fake_network_preflight.hpp"
#include "hypertron_ros2_bridge/network_preflight.hpp"

namespace hypertron_ros2_bridge {
namespace {

const NetworkExpectation kWiredExpectation{"eno1", "10.18.0.200/24",
                                           "10.18.0.100"};

TEST(NetworkPreflight, AcceptsExactHostAddressAndWiredRoute) {
  NetworkFacts facts{{"10.18.0.200/24"}, "eno1", "10.18.0.200"};
  const NetworkDecision decision = evaluate_network(facts, kWiredExpectation);
  EXPECT_TRUE(decision.ready) << decision.message;
  EXPECT_FALSE(decision.message.empty());
}

TEST(NetworkPreflight, RejectsRouteThroughWrongInterface) {
  NetworkFacts facts{{"10.18.0.200/24"}, "Meta", "10.18.0.200"};
  const NetworkDecision decision = evaluate_network(facts, kWiredExpectation);
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("Meta"), std::string::npos);
  EXPECT_NE(decision.message.find("eno1"), std::string::npos);
}

TEST(NetworkPreflight, RejectsMissingHostAddress) {
  NetworkFacts facts{{"192.168.100.106/23"}, "eno1", "10.18.0.200"};
  const NetworkDecision decision = evaluate_network(facts, kWiredExpectation);
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("10.18.0.200/24"), std::string::npos);
}

TEST(NetworkPreflight, RejectsWrongAddressPrefix) {
  NetworkFacts facts{{"10.18.0.200/16"}, "eno1", "10.18.0.200"};
  EXPECT_FALSE(evaluate_network(facts, kWiredExpectation).ready);
}

TEST(NetworkPreflight, RejectsHostAddressWithoutExpectedPrefix) {
  // Exact CIDR: expected "10.18.0.200/24" must not accept a bare
  // "10.18.0.200" fact, even though the address part matches.
  NetworkFacts facts{{"10.18.0.200"}, "eno1", "10.18.0.200"};
  const NetworkDecision decision = evaluate_network(facts, kWiredExpectation);
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("10.18.0.200/24"), std::string::npos);
}

TEST(NetworkPreflight, RejectsUnexpectedPrefixWhenExpectationHasNone) {
  // The exact-CIDR rule is symmetric: a fact carrying a prefix must not
  // match an expectation without one.
  NetworkFacts facts{{"10.18.0.200/24"}, "eno1", "10.18.0.200"};
  const NetworkDecision decision =
      evaluate_network(facts, {"eno1", "10.18.0.200", "10.18.0.100"});
  EXPECT_FALSE(decision.ready);
}

TEST(NetworkPreflight, RejectsRouteMissingSourceAddress) {
  NetworkFacts facts{{"10.18.0.200/24"}, "eno1", ""};
  const NetworkDecision decision = evaluate_network(facts, kWiredExpectation);
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("source address"), std::string::npos);
}

TEST(NetworkPreflight, RejectsMissingRouteWithReadableError) {
  NetworkFacts facts{{"10.18.0.200/24"}, "", ""};
  facts.route_error = "ip route get failed: Network is unreachable";
  const NetworkDecision decision = evaluate_network(facts, kWiredExpectation);
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("Network is unreachable"), std::string::npos);
}

TEST(NetworkPreflight, ReportsAddressProbeFailureReadably) {
  NetworkFacts facts;
  facts.addresses_error = "ip addr show exited with status 1";
  facts.route_interface = "eno1";
  facts.route_source = "10.18.0.200";
  const NetworkDecision decision = evaluate_network(facts, kWiredExpectation);
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("ip addr show exited with status 1"),
            std::string::npos);
}

TEST(NetworkPreflight, RejectsMalformedExpectedAddress) {
  NetworkFacts facts{{"10.18.0.200/24"}, "eno1", "10.18.0.200"};
  const NetworkDecision decision =
      evaluate_network(facts, {"eno1", "not-an-ip", "10.18.0.100"});
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("not-an-ip"), std::string::npos);
}

TEST(NetworkPreflight, ParsesIpAddrShowOutput) {
  std::vector<std::string> addresses;
  EXPECT_TRUE(parse_ip_addr_output(
      "1: lo    inet 127.0.0.1/8 scope host lo\\       "
      "valid_lft forever preferred_lft forever\n"
      "3: eno1  inet 10.18.0.200/24 brd 10.18.0.255 scope global eno1\\       "
      "valid_lft forever preferred_lft forever\n",
      addresses));
  ASSERT_EQ(addresses.size(), 2U);
  EXPECT_EQ(addresses[0], "127.0.0.1/8");
  EXPECT_EQ(addresses[1], "10.18.0.200/24");
}

TEST(NetworkPreflight, ParsesIpRouteGetOutput) {
  std::string device;
  std::string source;
  EXPECT_TRUE(parse_ip_route_get_output(
      "10.18.0.100 dev eno1 src 10.18.0.200 uid 1000 \n    cache\n", device,
      source));
  EXPECT_EQ(device, "eno1");
  EXPECT_EQ(source, "10.18.0.200");
}

// Scripted read-only command runner: serves canned output without touching
// the system and records every argv vector it receives. State lives behind a
// shared_ptr because LinuxNetworkPreflight copies its runner into a
// std::function.
class ScriptedRunner {
 public:
  struct State {
    std::string addr_output;
    int addr_rc{0};
    std::string route_output;
    int route_rc{0};
    std::string route_stderr;
    std::vector<std::vector<std::string>> invoked;
  };

  ScriptedRunner() : state_(std::make_shared<State>()) {}

  int operator()(const std::vector<std::string>& args, std::string& out,
                 std::string& err) {
    state_->invoked.push_back(args);
    if (args.size() == 5 && args[0] == "ip" && args[1] == "-4" &&
        args[2] == "-o" && args[3] == "addr" && args[4] == "show") {
      out = state_->addr_output;
      return state_->addr_rc;
    }
    if (args.size() == 6 && args[0] == "ip" && args[1] == "-4" &&
        args[2] == "-o" && args[3] == "route" && args[4] == "get") {
      out = state_->route_output;
      err = state_->route_stderr;
      return state_->route_rc;
    }
    return 127;
  }

  std::shared_ptr<State> state_;
};

TEST(NetworkPreflight, LinuxGatherPassesOnWiredFacts) {
  ScriptedRunner runner;
  runner.state_->addr_output =
      "3: eno1  inet 10.18.0.200/24 brd 10.18.0.255 scope global eno1\n";
  runner.state_->route_output = "10.18.0.100 dev eno1 src 10.18.0.200 uid 1000 \n"
                        "    cache\n";
  LinuxNetworkPreflight preflight(runner);
  const NetworkDecision decision = preflight.check(kWiredExpectation);
  EXPECT_TRUE(decision.ready) << decision.message;
  ASSERT_EQ(runner.state_->invoked.size(), 2U);
}

TEST(NetworkPreflight, LinuxGatherReportsUnreachableRoute) {
  ScriptedRunner runner;
  runner.state_->addr_output =
      "3: eno1  inet 10.18.0.200/24 brd 10.18.0.255 scope global eno1\n";
  runner.state_->route_rc = 2;
  runner.state_->route_stderr = "RTNETLINK answers: Network is unreachable";
  LinuxNetworkPreflight preflight(runner);
  const NetworkDecision decision = preflight.check(kWiredExpectation);
  EXPECT_FALSE(decision.ready);
  EXPECT_NE(decision.message.find("Network is unreachable"),
            std::string::npos);
}

TEST(NetworkPreflight, PassesArgumentsAsSingleVectorElements) {
  // A shell would split or interpret this string; a vector exec must pass it
  // through verbatim as one argv element.
  const std::string hostile = "10.18.0.100; touch /tmp/opencode/owned";
  ScriptedRunner runner;
  LinuxNetworkPreflight preflight(runner);
  preflight.check({"eno1", "10.18.0.200/24", hostile});
  ASSERT_FALSE(runner.state_->invoked.empty());
  const auto& args = runner.state_->invoked.back();
  ASSERT_EQ(args.size(), 6U);
  EXPECT_EQ(args[3], "route");
  EXPECT_EQ(args[4], "get");
  EXPECT_EQ(args[5], hostile);
}

TEST(NetworkPreflight, RealRunnerReadsSystemReadOnlyWithoutCrashing) {
  // Exercises the default fork/execvp runner against the host's real `ip`
  // output. Strictly read-only; passes on any host state because readiness
  // is host-dependent.
  LinuxNetworkPreflight preflight;
  const NetworkDecision decision = preflight.check(kWiredExpectation);
  EXPECT_FALSE(decision.message.empty());
  SUCCEED() << "decision: " << decision.message;
}

TEST(NetworkPreflight, FakePreflightReturnsProgrammedDecisionAndRecordsCalls) {
  test::FakeNetworkPreflight fake;
  fake.next_decision.ready = true;
  fake.next_decision.message = "programmed";
  const NetworkDecision decision = fake.check(kWiredExpectation);
  EXPECT_TRUE(decision.ready);
  EXPECT_EQ(decision.message, "programmed");
  ASSERT_EQ(fake.expected_seen.size(), 1U);
  EXPECT_EQ(fake.expected_seen.front().interface, "eno1");
  EXPECT_EQ(fake.expected_seen.front().robot_address, "10.18.0.100");
}

TEST(NetworkPreflight, ExecuteCommandReturnsNonZeroExitStatus) {
  std::string out;
  std::string err;
  const int rc = execute_readonly_command({"/bin/false"}, out, err, 5000);
  EXPECT_EQ(rc, 1);
}

TEST(NetworkPreflight, ExecuteCommandReportsExecFailure) {
  std::string out;
  std::string err;
  const int rc =
      execute_readonly_command({"/nonexistent-htbr-cmd"}, out, err, 5000);
  EXPECT_EQ(rc, 127);
}

TEST(NetworkPreflight, ExecuteCommandKillsChildOnTimeout) {
  const auto start = std::chrono::steady_clock::now();
  std::string out;
  std::string err;
  const int rc = execute_readonly_command({"sleep", "5"}, out, err, 150);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(rc, 124);
  EXPECT_NE(err.find("timed out"), std::string::npos);
  EXPECT_LT(elapsed, std::chrono::seconds(2));
}

TEST(NetworkPreflight, ExecuteCommandCapturesOutputs) {
  std::string out;
  std::string err;
  const int rc = execute_readonly_command({"echo", "hello"}, out, err, 5000);
  EXPECT_EQ(rc, 0);
  EXPECT_EQ(out, "hello\n");
  std::string ls_out;
  std::string ls_err;
  const int ls_rc =
      execute_readonly_command({"/bin/ls", "/nonexistent-htbr-path"},
                               ls_out, ls_err, 5000);
  EXPECT_EQ(ls_rc, 2);
  // The command path always appears in stderr regardless of locale.
  EXPECT_NE(ls_err.find("nonexistent-htbr-path"), std::string::npos);
  EXPECT_FALSE(ls_err.empty());
}

TEST(NetworkPreflight, ParsesAddrOutputWithVaryingTokens) {
  std::vector<std::string> addresses;
  EXPECT_TRUE(parse_ip_addr_output(
      "2: eth0    inet 192.168.1.5/24 brd 192.168.1.255 scope global dynamic\n"
      "3: eno1    inet 10.18.0.200/24 brd 10.18.0.255 scope global eno1\n",
      addresses));
  ASSERT_EQ(addresses.size(), 2U);
  EXPECT_EQ(addresses[0], "192.168.1.5/24");
  EXPECT_EQ(addresses[1], "10.18.0.200/24");
}

TEST(NetworkPreflight, ParsesWithIrregularWhitespace) {
  std::vector<std::string> addresses;
  EXPECT_TRUE(parse_ip_addr_output(
      "  1: lo    inet 127.0.0.1/8   scope host lo\n"
      "  3: eno1\tinet 10.18.0.200/24\tbrd 10.18.0.255\n",
      addresses));
  ASSERT_EQ(addresses.size(), 2U);
  EXPECT_EQ(addresses[1], "10.18.0.200/24");
}

TEST(NetworkPreflight, ParsesRouteGetOutputWithExtraAttributes) {
  std::string device;
  std::string source;
  EXPECT_TRUE(parse_ip_route_get_output(
      "10.18.0.100 via 10.0.0.1 dev eno1 src 10.18.0.200 uid 1000 proto "
      "dhcp\n",
      device, source));
  EXPECT_EQ(device, "eno1");
  EXPECT_EQ(source, "10.18.0.200");
}

TEST(NetworkPreflight, ParsesRouteGetWithSourceBeforeDevice) {
  std::string device;
  std::string source;
  EXPECT_TRUE(parse_ip_route_get_output(
      "10.18.0.100 src 10.18.0.200 dev eno1 uid 1000\n", device, source));
  EXPECT_EQ(device, "eno1");
  EXPECT_EQ(source, "10.18.0.200");
}

TEST(NetworkPreflight, ParsesMultiLineRouteGetOutput) {
  std::string device;
  std::string source;
  EXPECT_TRUE(parse_ip_route_get_output(
      "10.18.0.100 dev eno1 src 10.18.0.200 uid 1000\n"
      "    cache\n"
      "    src 10.18.0.200\n",
      device, source));
  EXPECT_EQ(device, "eno1");
  EXPECT_EQ(source, "10.18.0.200");
}

TEST(NetworkPreflight, RejectsRouteGetOutputMissingSource) {
  std::string device;
  std::string source;
  EXPECT_FALSE(parse_ip_route_get_output(
      "10.18.0.100 dev eno1 uid 1000\n", device, source));
}

TEST(NetworkPreflight, RejectsRouteGetOutputWithoutDevice) {
  std::string device;
  std::string source;
  EXPECT_FALSE(parse_ip_route_get_output(
      "10.18.0.100 via 10.0.0.1 src 10.18.0.200 uid 1000\n", device,
      source));
}

TEST(NetworkPreflight, ExecuteCommandTimesOutEvenAfterChildClosedStreams) {
  // The child closes both streams and then hangs: the timeout must cover
  // the ENTIRE child lifetime (not only the pipe drain). The old executor
  // blocked forever in waitpid after both pipes hit EOF.
  const auto start = std::chrono::steady_clock::now();
  std::string out;
  std::string err;
  const int rc = execute_readonly_command(
      {"/bin/sh", "-c", "exec 1>&-; exec 2>&-; sleep 30"}, out, err, 150);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(rc, 124);
  EXPECT_NE(err.find("timed out"), std::string::npos);
  EXPECT_LT(elapsed, std::chrono::seconds(3));
}

TEST(NetworkPreflight, ExecuteCommandTimesOutWhenChildWritesThenHangs) {
  // The child writes one line and keeps the stream OPEN while hanging:
  // the read path must never block past the deadline (O_NONBLOCK + EAGAIN
  // back to the poll loop). The old executor blocked forever in the inner
  // read loop.
  const auto start = std::chrono::steady_clock::now();
  std::string out;
  std::string err;
  const int rc = execute_readonly_command(
      {"/bin/sh", "-c", "echo hi; sleep 30"}, out, err, 150);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(rc, 124);
  EXPECT_EQ(out, "hi\n");
  EXPECT_NE(err.find("timed out"), std::string::npos);
  EXPECT_LT(elapsed, std::chrono::seconds(3));
}

TEST(NetworkPreflight, ExecuteCommandNormalExitIsNotReportedAsTimeout) {
  // A child that closes its streams and exits normally must return its
  // exit status, never 124 — even when the pipe EOF is observed before the
  // exit is reaped.
  std::string out;
  std::string err;
  const int rc = execute_readonly_command({"/bin/sh", "-c", "exit 0"}, out,
                                          err, 150);
  EXPECT_EQ(rc, 0);
  EXPECT_EQ(err.find("timed out"), std::string::npos);
}

TEST(NetworkPreflight, DefaultRunnerCompletesIpCommandsQuickly) {
  // The default runner drives the two real `ip` probes used by preflight;
  // both must finish well below the command timeout.
  const auto start = std::chrono::steady_clock::now();
  LinuxNetworkPreflight preflight;
  const NetworkDecision decision = preflight.check(kWiredExpectation);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, std::chrono::seconds(2));
  EXPECT_FALSE(decision.message.empty());
}

}  // namespace
}  // namespace hypertron_ros2_bridge
