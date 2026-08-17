#include "hypertron_ros2_bridge/network_preflight.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <chrono>
#include <cstring>
#include <optional>
#include <sstream>
#include <utility>

namespace hypertron_ros2_bridge {

namespace {

struct ParsedAddress {
  std::string address;
  bool has_prefix{false};
  int prefix{};
};

// Parses "a.b.c.d[/nn]"; returns nullopt when malformed.
std::optional<ParsedAddress> parse_address(const std::string& cidr) {
  ParsedAddress parsed;
  std::string rest = cidr;
  const std::size_t slash = rest.find('/');
  if (slash != std::string::npos) {
    const std::string prefix_text = rest.substr(slash + 1);
    if (prefix_text.empty()) {
      return std::nullopt;
    }
    try {
      std::size_t consumed = 0;
      parsed.prefix = std::stoi(prefix_text, &consumed);
      if (consumed != prefix_text.size() || parsed.prefix < 0 ||
          parsed.prefix > 32) {
        return std::nullopt;
      }
    } catch (const std::exception&) {
      return std::nullopt;
    }
    parsed.has_prefix = true;
    rest = rest.substr(0, slash);
  }
  if (rest.empty()) {
    return std::nullopt;
  }
  in_addr binary{};
  if (inet_pton(AF_INET, rest.c_str(), &binary) != 1) {
    return std::nullopt;
  }
  parsed.address = rest;
  return parsed;
}

std::string join_failures(const std::string& first, const std::string& second) {
  if (first.empty()) {
    return second;
  }
  if (second.empty()) {
    return first;
  }
  return first + "; " + second;
}

}  // namespace

NetworkDecision evaluate_network(const NetworkFacts& facts,
                                 const NetworkExpectation& expected) {
  NetworkDecision decision;

  const auto expected_host = parse_address(expected.host_address);
  if (!expected_host.has_value()) {
    decision.message = "expected host address '" + expected.host_address +
                       "' is not a valid IPv4 address or CIDR";
    return decision;
  }

  std::string address_problem;
  if (!facts.addresses_error.empty()) {
    address_problem = "could not read host addresses: " + facts.addresses_error;
  } else {
    bool found = false;
    for (const std::string& entry : facts.addresses) {
      const auto parsed = parse_address(entry);
      if (!parsed.has_value() || parsed->address != expected_host->address) {
        continue;
      }
      // Exact CIDR match: prefix presence and value must both agree.
      if (parsed->has_prefix != expected_host->has_prefix) {
        continue;
      }
      if (expected_host->has_prefix &&
          parsed->prefix != expected_host->prefix) {
        continue;
      }
      found = true;
      break;
    }
    if (!found) {
      address_problem = "host address '" + expected.host_address +
                        "' not found on any interface";
    }
  }

  std::string route_problem;
  if (!facts.route_error.empty()) {
    route_problem = "could not determine route to '" + expected.robot_address +
                    "': " + facts.route_error;
  } else if (facts.route_interface.empty()) {
    route_problem =
        "no route to robot address '" + expected.robot_address + "'";
  } else if (facts.route_interface != expected.interface) {
    route_problem = "route to '" + expected.robot_address +
                    "' uses interface '" + facts.route_interface +
                    "', expected '" + expected.interface + "'";
  } else if (facts.route_source != expected_host->address) {
    route_problem = "route to '" + expected.robot_address +
                    "' uses source address '" + facts.route_source +
                    "', expected '" + expected_host->address + "'";
  }

  const std::string problem = join_failures(address_problem, route_problem);
  if (problem.empty()) {
    decision.ready = true;
    decision.message = "network preflight ok: " + expected.host_address +
                       " on " + expected.interface + ", route to " +
                       expected.robot_address + " via " + expected.interface;
  } else {
    decision.message = problem;
  }
  return decision;
}

bool parse_ip_addr_output(const std::string& output,
                          std::vector<std::string>& addresses) {
  std::istringstream lines(output);
  std::string line;
  bool found_inet = false;
  while (std::getline(lines, line)) {
    std::istringstream tokens(line);
    std::string token;
    while (tokens >> token) {
      if (token == "inet") {
        std::string cidr;
        if (tokens >> cidr) {
          addresses.push_back(cidr);
          found_inet = true;
        }
      }
    }
  }
  return found_inet;
}

bool parse_ip_route_get_output(const std::string& output, std::string& device,
                               std::string& source) {
  std::istringstream lines(output);
  std::string line;
  while (std::getline(lines, line)) {
    std::istringstream tokens(line);
    std::string token;
    std::string dev;
    std::string src;
    while (tokens >> token) {
      if (token == "dev") {
        tokens >> dev;
      } else if (token == "src") {
        tokens >> src;
      }
    }
    if (!dev.empty() && !src.empty()) {
      device = dev;
      source = src;
      return true;
    }
  }
  return false;
}

namespace {

// RAII for pipe ends: every exit path (including fork failure) closes the
// descriptors exactly once.
struct FdGuard {
  int fd;
  explicit FdGuard(int f) : fd(f) {}
  ~FdGuard() {
    if (fd >= 0) {
      close(fd);
    }
  }
  FdGuard(const FdGuard&) = delete;
  FdGuard& operator=(const FdGuard&) = delete;
  // Detaches the descriptor so the destructor does not close it.
  int release() {
    const int f = fd;
    fd = -1;
    return f;
  }
};

// Creates a pipe with both ends marked close-on-exec so no fd leaks into a
// successfully exec'd child. Prefers pipe2(O_CLOEXEC); falls back to pipe +
// fcntl for older kernels.
int make_cloexec_pipe(int fds[2]) {
#ifdef O_CLOEXEC
  if (pipe2(fds, O_CLOEXEC) == 0) {
    return 0;
  }
  if (errno != ENOSYS && errno != EINVAL) {
    return -1;
  }
#endif
  if (pipe(fds) != 0) {
    return -1;
  }
  // Both ends must be close-on-exec so no fd leaks into a successfully
  // exec'd child. A failed fcntl leaves the pipe in an unsafe state: close
  // both ends and report the failure.
  if (fcntl(fds[0], F_SETFD, FD_CLOEXEC) < 0 ||
      fcntl(fds[1], F_SETFD, FD_CLOEXEC) < 0) {
    close(fds[0]);
    close(fds[1]);
    fds[0] = -1;
    fds[1] = -1;
    return -1;
  }
  return 0;
}

}  // namespace

// Runs a command via fork/execvp with every argument passed as a separate
// vector element. No shell is involved, so no argument can be split,
// interpreted, or injected. Both output streams are captured and drained
// with poll() so neither pipe can deadlock the child. The poll is bounded
// by timeout_ms; on expiry the child is killed (SIGKILL) and reaped, a
// readable note is appended to stderr_text, and 124 is returned, so a
// hung command can never block the runtime thread forever. fork/dup2
// failures close every pipe end via FdGuard and report -1; a non-zero
// child exit status is propagated as-is (never misreported as success).
int execute_readonly_command(const std::vector<std::string>& args,
                             std::string& stdout_text,
                             std::string& stderr_text, int timeout_ms) {
  if (timeout_ms <= 0) {
    timeout_ms = 5000;
  }
  if (args.empty()) {
    return 127;
  }
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (const std::string& arg : args) {
    argv.push_back(const_cast<char*>(arg.c_str()));
  }
  argv.push_back(nullptr);

  int stdout_pipe[2] = {-1, -1};
  int stderr_pipe[2] = {-1, -1};
  if (make_cloexec_pipe(stdout_pipe) != 0 || make_cloexec_pipe(stderr_pipe) != 0) {
    if (stdout_pipe[0] >= 0) close(stdout_pipe[0]);
    if (stdout_pipe[1] >= 0) close(stdout_pipe[1]);
    if (stderr_pipe[0] >= 0) close(stderr_pipe[0]);
    if (stderr_pipe[1] >= 0) close(stderr_pipe[1]);
    return -1;
  }
  // Guards own all four ends from here on; every subsequent failure path
  // (including fork failure) closes them exactly once.
  FdGuard out_r(stdout_pipe[0]);
  FdGuard out_w(stdout_pipe[1]);
  FdGuard err_r(stderr_pipe[0]);
  FdGuard err_w(stderr_pipe[1]);

  const pid_t pid = fork();
  if (pid < 0) {
    // fork failed: guards close all pipe ends.
    return -1;
  }
  if (pid == 0) {
    // Child: attach pipes and exec. Only async-signal-safe calls follow.
    // _exit never runs destructors, so no guard cleanup happens here.
    if (dup2(stdout_pipe[1], STDOUT_FILENO) < 0) {
      _exit(126);
    }
    if (dup2(stderr_pipe[1], STDERR_FILENO) < 0) {
      _exit(126);
    }
    close(stdout_pipe[0]);
    close(stdout_pipe[1]);
    close(stderr_pipe[0]);
    close(stderr_pipe[1]);
    execvp(argv[0], argv.data());
    _exit(127);  // exec failure (e.g. no such command)
  }

  // Parent: the write ends must close immediately, otherwise poll() on the
  // read ends never observes EOF (HUP). The read ends are set to
  // O_NONBLOCK so a child that writes a little and then hangs can never
  // block the drain: read() returns EAGAIN and the loop goes back to poll,
  // which remains bounded by the deadline.
  close(out_w.release());
  close(err_w.release());
  // Preserve the existing flags (F_GETFL) and add O_NONBLOCK (F_SETFL);
  // never overwrite unknown flags. If either call fails, the fd stays in
  // blocking mode and must NOT participate in the drain: close it via the
  // FdGuard on the error path below (kill + reap) and report a readable
  // failure instead of risking an unbounded read.
  const int stdout_flags = fcntl(stdout_pipe[0], F_GETFL, 0);
  const int stderr_flags = fcntl(stderr_pipe[0], F_GETFL, 0);
  const bool nonblock_ok =
      stdout_flags >= 0 && stderr_flags >= 0 &&
      fcntl(stdout_pipe[0], F_SETFL, stdout_flags | O_NONBLOCK) == 0 &&
      fcntl(stderr_pipe[0], F_SETFL, stderr_flags | O_NONBLOCK) == 0;
  if (!nonblock_ok) {
    // Error path: reap the child (FdGuard closes all pipe ends) and report
    // a readable failure; never drain with blocking-mode fds.
    kill(pid, SIGKILL);
    stderr_text += "\n[command execution failed: could not set "
                   "non-blocking mode on the output pipes; child killed]";
    int reap_status = 0;
    pid_t waited;
    do {
      waited = waitpid(pid, &reap_status, 0);
    } while (waited < 0 && errno == EINTR);
    return -1;
  }

  bool timed_out = false;
  bool fatal_error = false;
  int status = 0;
  {
    pollfd fds[2] = {{stdout_pipe[0], POLLIN, 0}, {stderr_pipe[0], POLLIN, 0}};
    std::string* sinks[2] = {&stdout_text, &stderr_text};
    bool open_fds[2] = {true, true};
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    char buffer[4096];
    // Unified lifetime loop: the deadline bounds the ENTIRE child
    // lifecycle, not just the pipe drain. Every iteration polls with the
    // remaining time, drains ready data (non-blocking), and reaps with
    // waitpid(WNOHANG) so a child that exits (or closes its streams and
    // hangs) can never block the caller past timeout_ms.
    for (;;) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
      if (remaining.count() <= 0) {
        timed_out = true;
        break;
      }
      // When both read ends are already closed (EOF observed) but the
      // child has not exited yet, poll() on an all -1 array would return
      // immediately and busy-spin; use a short poll instead and let the
      // waitpid(WNOHANG) check below wait for the exit. A child that
      // closed its streams and exited is reaped on the first such check.
      const bool all_closed = !open_fds[0] && !open_fds[1];
      const int poll_timeout = all_closed
                                   ? std::min(30, static_cast<int>(remaining.count()))
                                   : static_cast<int>(remaining.count());
      const int ready = poll(fds, 2, poll_timeout);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        fatal_error = true;  // unrecoverable poll error
        break;
      }
      for (int i = 0; i < 2; ++i) {
        if (!open_fds[i]) {
          continue;
        }
        if ((fds[i].revents & (POLLIN | POLLHUP | POLLERR)) == 0) {
          continue;
        }
        for (;;) {
          const ssize_t count = read(fds[i].fd, buffer, sizeof(buffer));
          if (count > 0) {
            sinks[i]->append(buffer, static_cast<std::size_t>(count));
            continue;  // keep draining until EOF, EAGAIN, or error
          }
          if (count < 0 && errno == EINTR) {
            continue;  // retry the same fd
          }
          if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            break;  // no more data right now; back to the poll loop
          }
          close(fds[i].fd);
          fds[i].fd = -1;
          open_fds[i] = false;
          break;
        }
      }
      // Reap opportunistically: the child may have exited even before both
      // pipes hit EOF (e.g. it closed its streams and hung — the pipe EOF
      // is visible, but the child itself is still alive and must be
      // bounded by the deadline).
      const pid_t waited = waitpid(pid, &status, WNOHANG);
      if (waited == pid) {
        break;  // child exited normally (or by signal)
      }
      if (waited < 0 && errno != EINTR) {
        fatal_error = true;
        break;
      }
    }
  }

  if (timed_out || fatal_error) {
    // Bounded recovery: kill the child and reap it so no zombie remains,
    // then report the failure instead of blocking on an unresponsive
    // process. The child was not reaped inside the loop yet.
    kill(pid, SIGKILL);
    if (timed_out) {
      stderr_text += "\n[command timed out after ";
      stderr_text += std::to_string(timeout_ms);
      stderr_text += " ms; killed]";
    } else {
      stderr_text += "\n[command execution failed; child killed]";
    }
    pid_t waited;
    do {
      waited = waitpid(pid, &status, 0);
    } while (waited < 0 && errno == EINTR);
    if (waited < 0) {
      return -1;  // never misreport success on a failed wait
    }
    return timed_out ? 124 : -1;
  }

  // Normal exit: the loop already reaped the child via waitpid(WNOHANG),
  // so `status` is valid here. A second waitpid would fail with ECHILD.
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  if (WIFSIGNALED(status)) {
    return 128 + WTERMSIG(status);
  }
  return -1;
}

LinuxNetworkPreflight::LinuxNetworkPreflight(CommandRunner runner)
    : runner_(runner
                  ? std::move(runner)
                  : CommandRunner([](const std::vector<std::string>& args,
                                     std::string& out, std::string& err) {
                      return execute_readonly_command(args, out, err, 5000);
                    })) {}

LinuxNetworkPreflight::~LinuxNetworkPreflight() = default;

NetworkDecision LinuxNetworkPreflight::check(
    const NetworkExpectation& expected) {
  NetworkFacts facts;

  {
    std::string output;
    std::string error;
    const int rc = runner_({"ip", "-4", "-o", "addr", "show"}, output, error);
    if (rc != 0) {
      facts.addresses_error = "'ip -4 -o addr show' exited with status " +
                              std::to_string(rc) +
                              (error.empty() ? std::string() : ": " + error);
    } else if (!parse_ip_addr_output(output, facts.addresses)) {
      facts.addresses_error = "'ip -4 -o addr show' returned no IPv4 addresses";
    }
  }

  {
    std::string output;
    std::string error;
    const int rc = runner_(
        {"ip", "-4", "-o", "route", "get", expected.robot_address}, output,
        error);
    if (rc != 0) {
      facts.route_error = "'ip route get " + expected.robot_address +
                          "' exited with status " + std::to_string(rc) +
                          (error.empty() ? std::string() : ": " + error);
    } else {
      std::string device;
      std::string source;
      if (parse_ip_route_get_output(output, device, source)) {
        facts.route_interface = device;
        facts.route_source = source;
      } else {
        facts.route_error = "'ip route get " + expected.robot_address +
                            "' produced unparseable output";
      }
    }
  }

  return evaluate_network(facts, expected);
}

}  // namespace hypertron_ros2_bridge
