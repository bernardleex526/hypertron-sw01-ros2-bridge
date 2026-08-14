#pragma once

#include <functional>
#include <string>
#include <vector>

namespace hypertron_ros2_bridge {

// Read-only snapshot of the host network facts relevant to the robot link.
// No field here changes any network configuration.
struct NetworkFacts {
  // Host addresses as "a.b.c.d/prefix" strings, e.g. "10.18.0.200/24".
  std::vector<std::string> addresses{};
  // Interface carrying the route to the robot address ("" when absent).
  std::string route_interface{};
  // Source address of that route, e.g. "10.18.0.200".
  std::string route_source{};
  // Readable probe failures; non-empty means the probe itself failed.
  std::string addresses_error{};
  std::string route_error{};
};

struct NetworkExpectation {
  // Wired interface that must carry the robot route, e.g. "eno1".
  std::string interface;
  // Host address with prefix that must exist locally, e.g. "10.18.0.200/24".
  std::string host_address;
  // Robot address the route must reach, e.g. "10.18.0.100".
  std::string robot_address;
};

struct NetworkDecision {
  bool ready{false};
  // Human-readable reason; empty is invalid, readable in every other case.
  std::string message;
};

// Pure fact evaluation with no system access. Every rejection produces a
// readable message.
NetworkDecision evaluate_network(const NetworkFacts& facts,
                                 const NetworkExpectation& expected);

// Parsers over the output of the read-only `ip` command so tests can feed
// canned output without touching the system.
// parse_ip_addr_output: true when at least one inet entry was found.
bool parse_ip_addr_output(const std::string& output,
                          std::vector<std::string>& addresses);
// parse_ip_route_get_output: true when a dev/source pair was found.
bool parse_ip_route_get_output(const std::string& output, std::string& device,
                               std::string& source);

// Read-only command executor used by LinuxNetworkPreflight. Executes argv
// verbatim via fork/execvp (no shell), captures both streams with bounded
// polling, and returns the child's exit status (WEXITSTATUS, or
// 128+signal). On timeout the child is killed and reaped, stderr_text gains
// a readable "timed out" note, and 124 is returned. Never blocks the
// caller for longer than timeout_ms (<=0 means the 5000 ms default).
int execute_readonly_command(const std::vector<std::string>& args,
                             std::string& stdout_text,
                             std::string& stderr_text, int timeout_ms = 5000);

// Injectable host-network validation seam.
class INetworkPreflight {
 public:
  virtual ~INetworkPreflight() = default;
  virtual NetworkDecision check(const NetworkExpectation& expected) = 0;
};

// Read-only Linux implementation. Gathers address and route facts via `ip`
// and never modifies the system network configuration.
class LinuxNetworkPreflight final : public INetworkPreflight {
 public:
  // Executes argv verbatim and fills the output captures, returning the exit
  // status. Defaults to a fork/execvp runner that never invokes a shell;
  // injectable for tests.
  using CommandRunner =
      std::function<int(const std::vector<std::string>& args,
                        std::string& stdout_text, std::string& stderr_text)>;

  explicit LinuxNetworkPreflight(CommandRunner runner = {});
  ~LinuxNetworkPreflight() override;
  LinuxNetworkPreflight(const LinuxNetworkPreflight&) = delete;
  LinuxNetworkPreflight& operator=(const LinuxNetworkPreflight&) = delete;

  NetworkDecision check(const NetworkExpectation& expected) override;

 private:
  CommandRunner runner_;
};

}  // namespace hypertron_ros2_bridge
