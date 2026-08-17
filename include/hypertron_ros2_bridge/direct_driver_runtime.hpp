#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "hypertron_ros2_bridge/astrall_sdk.hpp"
#include "hypertron_ros2_bridge/network_preflight.hpp"
#include "hypertron_ros2_bridge/robot_controller.hpp"
#include "hypertron_ros2_bridge/thread_safe_queue.hpp"

namespace hypertron_ros2_bridge {

// Configuration for the direct driver runtime. The network fields form the
// preflight expectation checked before every connection attempt.
struct RuntimeConfig {
  // Wired interface that must carry the robot route, e.g. "eno1".
  std::string interface;
  // Host address with prefix that must exist locally, e.g. "10.18.0.200/24".
  std::string host_address;
  // Robot address the route must reach, e.g. "10.18.0.100".
  std::string robot_address;

  // SDK call timeouts in milliseconds.
  std::uint32_t init_timeout_ms{60000U};
  std::uint32_t sdk_call_timeout_ms{5000U};
  // Timeout of one SDK heartbeat call; the worker blocks on the call at
  // most this long before a failure can end the session.
  std::uint32_t heartbeat_call_timeout_ms{500U};
  // Consecutive failed heartbeat calls that end a connected session. The
  // default (1) matches the design contract: a single heartbeat failure
  // clears pending motion and mode work and reconnects.
  std::uint32_t heartbeat_max_failures{1U};

  // Bounded queue for low-rate commands (authority and mode requests). A
  // full queue rejects the newest request with an immediate failure instead
  // of ever blocking the caller.
  std::size_t queue_capacity{16};

  // Connected-loop periods.
  std::chrono::milliseconds heartbeat_period{100};
  std::chrono::milliseconds motion_refresh_period{20};
  std::chrono::milliseconds deadman_ms{100};
  std::chrono::milliseconds mode_timeout_ms{5000};
  std::chrono::milliseconds state_poll_period{500};

  // Telemetry subscription rates.
  SubscriptionFrequency imu_freq{SubscriptionFrequency::Hz50};
  SubscriptionFrequency sport_freq{SubscriptionFrequency::Hz50};
  // When true the connected session also opens the LIDAR discard subscription
  // (enables the robot's UDP 6100/6101 push streams). Disabled sessions never
  // call subscribe_lidar.
  bool enable_lidar_stream{true};

  // Bounded reconnect backoff, doubled after every failed attempt up to the
  // maximum.
  std::chrono::milliseconds reconnect_initial_delay_ms{1000};
  std::chrono::milliseconds reconnect_max_delay_ms{30000};
};

// Event severities used with RuntimeObserver::on_event.
constexpr int kRuntimeEventInfo = 0;
constexpr int kRuntimeEventWarning = 1;
constexpr int kRuntimeEventError = 2;

// Telemetry and lifecycle observer for the runtime. Callbacks may arrive on
// the runtime worker thread or on a vendor SDK thread; implementations must
// not block and must copy anything they retain.
class RuntimeObserver {
 public:
  virtual ~RuntimeObserver() = default;

  // A state snapshot delivered by the worker's poll loop.
  virtual void on_state(const SdkSnapshot& snapshot) = 0;
  // An IMU sample delivered by the SDK subscription callback.
  virtual void on_imu(const ImuSample& sample) = 0;
  // A sport-state sample delivered by the SDK subscription callback.
  virtual void on_sport(const SportSample& sample) = 0;
  // The SDK status callback values (linked, control authority).
  virtual void on_status(bool linked, bool control_authority) = 0;
  // Lifecycle notification with a kRuntimeEvent* severity.
  virtual void on_event(int severity, std::string message) = 0;
};

// Owns the SDK lifecycle on a single worker thread: preflight, init,
// telemetry subscriptions, heartbeat, state polling, and bounded reconnects.
// Startup and every reconnection are non-actuating by construction: the
// worker executes an authority or mode command only after it was explicitly
// enqueued through request_authority() or request_mode(), and it moves the
// robot only through the three gated paths below:
//  - submit_velocity() commands are accepted by the RobotController safety
//    gate and re-sent by the worker at the motion refresh period;
//  - trigger_estop(true) commands zero velocity plus damping mode with
//    priority over every queued command when the SDK link and control
//    authority are present;
//  - stop() sends the same zero-plus-damping sequence before tearing the
//    session down, also only when the link and authority are present.
// Commands are serialized on the worker through a bounded queue, so callers
// never block beyond the enqueue itself.
//
// Failure handling: a failed preflight/init/subscribe, a heartbeat failure
// (one failed call by default, up to the configured consecutive-failure
// threshold), or an SDK-reported link drop clears the controller status and
// the runtime-side link/authority snapshot cache and invalidates the
// connection (preserving the software emergency-stop latch), fails every
// queued or in-flight command with a disconnected result, deinitializes the
// SDK, waits a bounded backoff, and retries from preflight. Reconnection
// never restores authority, mode, or velocity, and the driver readiness
// gate re-arms only when the first snapshot of the new connection reports
// an SDK link — never on stale state from the session that just ended.
//
// Status handling: every SDK status callback (possibly on a vendor thread)
// updates the controller status immediately from {linked, authority, last
// polled sport status, last polled error code} and requests an immediate
// motion tick, so a lost link or lost authority closes the safety gate and
// stops a dispatched velocity without waiting for the next state poll. The
// callback belongs to the connection attempt in progress: a status report
// delivered during init or the subscriptions counts for the session that
// follows, and a linked=false report latches the end of that session even
// when a later report in the same session re-links. State polls apply only
// the snapshot's sport status and error code on top of the callback-cached
// link/authority, so an older snapshot can never overwrite a newer
// authority or link loss with stale values.
//
// Threading: the worker is the only thread that calls SDK entry points or
// touches the RobotController state machine; velocity submission and estop
// requests only touch the thread-safe controller. Every wait (backoff and
// connected-loop deadlines) is interruptible, so stop() always returns
// promptly, even when it is called from inside an in-flight callback on the
// worker thread or from inside an observer callback dispatched on a vendor
// SDK thread (in both cases it flags the shutdown and returns without
// joining itself; the worker exits on its own). stop() is additionally
// serialized by a dedicated stop mutex, so concurrent stop() calls from
// plain threads join the worker exactly once and all of them return.
// Commands are accepted only while the runtime is inside a live connected
// session with a confirmed SDK link; otherwise the returned future is
// already resolved with a failure when the caller receives it.
//
// Lifecycle: a single start()/stop() cycle. The destructor joins the worker
// and then deinitializes the SDK. It must never run on a vendor callback
// thread: destroying the runtime there would join (or race) the worker
// while it is still inside an in-flight SDK call and could destroy the SDK
// on the vendor call stack.
class DirectDriverRuntime {
 public:
  DirectDriverRuntime(std::unique_ptr<IAstrallSdk> sdk,
                      std::unique_ptr<INetworkPreflight> preflight,
                      IMonotonicClock& clock, RuntimeConfig config,
                      RuntimeObserver* observer);
  ~DirectDriverRuntime();

  DirectDriverRuntime(const DirectDriverRuntime&) = delete;
  DirectDriverRuntime& operator=(const DirectDriverRuntime&) = delete;

  // Starts the worker thread and returns immediately; no SDK, network, or
  // controller call is made from the caller's thread. A no-op when the
  // runtime is already running or has been stopped.
  void start();
  // Requests shutdown, wakes every wait, and joins the worker. Idempotent.
  // When the SDK link and control authority are present the worker first
  // sends zero velocity and damping mode (each bounded by the SDK call
  // timeout) before tearing the session down; without authority no
  // actuating SDK call is made. Safe to call from any thread, including
  // from inside an in-flight callback running on the worker thread and
  // from inside an observer callback dispatched on a vendor SDK thread: in
  // both cases the shutdown flag is set, the worker is woken, and stop()
  // returns without joining (the worker exits on its own; the destructor
  // joins from the owning thread). Concurrent stop() calls from plain
  // threads are serialized by an internal stop mutex: the worker is joined
  // exactly once and every caller returns.
  void stop() noexcept;

  // Latest telemetry copies delivered by the SDK subscription callbacks.
  ImuSample latest_imu() const;
  SportSample latest_sport() const;

  // Number of observer callback invocations that threw (caught and counted,
  // never rethrown) since construction. Exposed so tests can prove a
  // throwing observer is isolated and the worker keeps running.
  std::uint64_t observer_error_count() const;

  // Non-blocking, latest-value velocity command. The RobotController safety
  // gate (driver readiness, SDK link, control authority, no system error,
  // all-terrain Move state, no pending mode transition, no emergency-stop
  // latch) decides acceptance; a rejected command is dropped with a warning
  // event and never reaches the SDK. An accepted command overwrites any
  // previously accepted one and is re-sent by the worker at the motion
  // refresh period while it stays within the command deadman; when the
  // deadman expires the worker sends one zero-velocity move.
  void submit_velocity(const Velocity& velocity);

  // Software emergency stop. engage=true latches the stop in the controller
  // immediately (synchronous), clears pending mode transitions and queued
  // commands through a worker-side mark, and - when the SDK link and
  // control authority are present - commands zero velocity and damping
  // mode with priority over every queued command before the returned future
  // is resolved by the worker. When the runtime is disconnected the latch
  // still happens and the future is already failed when returned. engage=
  // false only releases the software latch (controller_.clear_estop()); no
  // velocity or mode is replayed and the returned future is resolved
  // immediately.
  std::future<Result> trigger_estop(bool engage);

  // Low-rate commands executed serially by the worker. Each returns a future
  // resolved by the worker with the mapped SDK Result; the calling thread
  // never blocks beyond the bounded enqueue. When the runtime is
  // disconnected (never started, stopped, between sessions, or SDK link
  // down), the future is already failed when returned (kSdkResNotInit with a
  // "disconnected" message). When the bounded queue is full the future fails
  // immediately with kSdkResFailed.
  //
  // request_authority(true) requests SDK control authority;
  // request_authority(false) transfers authority back to the remote
  // controller (the SDK 1.0.7 joystick target).
  std::future<Result> request_authority(bool sdk);
  // Maps a documented mode name (damping/stand/down/move/auto_charge/
  // exit_charge/recover/recovery) through the RobotController safety gate.
  // An invalid name or a gate rejection (transition pending, readiness not
  // armed, disconnected, system error, no authority, e-stop latched) fails
  // the future immediately. An accepted request is executed by the worker:
  // sdk->set_mode() first, then the sport status is polled at the state poll
  // period until the mode's stable status appears or mode_timeout_ms
  // expires. The connected loop wakes at the transition deadline (even when
  // the state poll period is longer), so the timeout settles promptly at the
  // deadline instead of lagging behind the poll; the controller transition
  // completes in both cases.
  std::future<Result> request_mode(const std::string& name);

 private:
  struct Command {
    enum class Kind { RequestAuthority, RequestMode };
    Kind kind{Kind::RequestAuthority};
    bool authority_sdk{};
    std::uint16_t mode{};
    std::shared_ptr<std::promise<Result>> promise;
  };

  // Mode transition awaiting its stable sport status. Only ever touched by
  // the worker thread.
  struct ModeTransition {
    bool active{false};
    std::uint16_t stable_status{0};
    IMonotonicClock::time_point deadline{};
    std::shared_ptr<std::promise<Result>> promise;
  };

  void worker_main();
  void run_connected_loop();
  Result subscribe_telemetry();
  SdkCallbacks make_sdk_callbacks();
  void handle_status(bool linked, bool control_authority);
  void handle_imu(const ImuSample& sample);
  void handle_sport(const SportSample& sample);
  void notify_event(int severity, std::string message);
  bool stopping() const;
  std::chrono::milliseconds wait_backoff(std::chrono::milliseconds current);
  // Executes one dequeued command (authority passthrough, or set_mode plus
  // arming the sport-status transition watch).
  void execute_command(const Command& command);
  // Poll-time check of an armed transition: completes it with success when
  // the stable sport status is observed. A passed deadline is settled by
  // settle_mode_transition_timeout() - normally from the loop's deadline
  // wake, with the check here as the fallback when a poll observes the
  // deadline first. Both outcomes run complete_mode_transition() and
  // resolve the command promise.
  void check_mode_transition(const SdkSnapshot& snapshot,
                             IMonotonicClock::time_point now);
  // Worker-thread settlement of an armed mode transition whose deadline
  // passed: completes the controller transition and fails the command
  // promise with the timeout code. Runs from the connected loop when the
  // transition deadline arrives, independent of the state poll period.
  void settle_mode_transition_timeout();
  // Fails and clears the worker-side mode transition. When
  // complete_controller is true the controller transition is completed as a
  // failure as well; callers that are about to install a newer transition
  // pass false because the controller already owns the new request.
  void fail_mode_transition(const Result& result, bool complete_controller);
  // Worker-thread emergency-stop processing, run at the top of every
  // connected-loop iteration so it takes priority over queued commands:
  // fails every queued command and any in-flight mode transition, then -
  // when the SDK link and control authority are present - sends zero
  // velocity and damping mode, and finally resolves the estop promise.
  void handle_estop_if_requested();
  // Sends zero velocity followed by damping mode, each bounded by the SDK
  // call timeout; failures only produce warning events. Returns true when
  // both calls succeeded. Worker thread only.
  bool send_zero_and_damping();
  // Shutdown actuation used by the worker when a connected session ends
  // because stop() was requested: sends zero velocity plus damping mode
  // when the SDK link and control authority are present, and nothing
  // otherwise. Worker thread only.
  void actuate_stop_if_authorized();
  // Session teardown: fails every queued command and any in-flight
  // transition with a disconnected result. Caller must hold wake_mutex_.
  void fail_all_pending_locked();
  // Session teardown on every disconnect path (preflight/init/subscribe
  // failure, heartbeat failure, SDK-reported link drop): clears the
  // controller status and the runtime-side link/authority/snapshot caches
  // so no stale values from the ended session survive into the next one
  // (commands, estop and stop must all see false/false until the next
  // session's status callback reports otherwise), then invalidates the
  // connection (readiness, pending mode transitions, velocity). Worker
  // thread only.
  void invalidate_session();
  // Resolves a promise without ever throwing (the promise may already be
  // resolved when a session teardown races a caller-side failure).
  static void set_promise(
      const std::shared_ptr<std::promise<Result>>& promise,
      const Result& result) noexcept;

  std::unique_ptr<IAstrallSdk> sdk_;
  std::unique_ptr<INetworkPreflight> preflight_;
  IMonotonicClock& clock_;
  RuntimeConfig config_;
  RuntimeObserver* observer_;
  // Count of observer callback invocations that threw (caught and counted by
  // dispatch_observer_safely). Never exposed to the observer; only observable
  // via observer_error_count().
  std::atomic<std::uint64_t> observer_errors_{0};

  RobotController controller_;

  std::thread worker_;
  // Serializes the joinable check and the join in stop(), so concurrent
  // stop() calls from plain threads join the worker exactly once and all
  // of them return. The self-join / in-observer-callback early returns
  // happen while holding this lock; the destructor joins from the owning
  // thread afterwards.
  mutable std::mutex stop_mutex_;
  // Guards stop_requested_, sdk_linked_, control_authority_, connected_,
  // the snapshot caches, link_drop_latched_ and motion_refresh_due_, and
  // wakes the worker's waits. The SDK status callback (possibly on a vendor
  // thread) takes this lock briefly and notifies. Enqueuing a command and
  // the session-end drain also hold this lock while touching the bounded
  // command queue, so a command can never be pushed after the drain.
  mutable std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  bool stop_requested_{false};
  bool sdk_linked_{false};
  // True while the worker is inside a connected session. Commands are only
  // accepted while this is set together with sdk_linked_.
  bool connected_{false};
  // Latched by any SDK status report with linked=false during the current
  // connection attempt (cleared before every init, so a stale report from
  // an earlier phase can never end the session that follows). The
  // connected loop treats the latch as the session-end event, so a drop
  // that is followed by a re-link report in the same session still ends
  // the session.
  bool link_drop_latched_{false};
  // Latest control-authority value reported by the SDK status callback.
  bool control_authority_{false};
  // Sport status and error code of the last state poll, cached so a status
  // callback can feed the controller an up-to-date status without waiting
  // for the next poll. Guarded by wake_mutex_.
  std::uint16_t last_snapshot_sport_status_{0};
  std::uint32_t last_snapshot_error_code_{0};
  // Set by the status callback to request an immediate motion tick from
  // the worker, so a dispatched velocity is stopped without waiting for
  // the next motion-refresh deadline. Guarded by wake_mutex_.
  bool motion_refresh_due_{false};
  // Worker-side emergency-stop mark: set by trigger_estop(true), consumed
  // by the worker at the top of a connected-loop iteration. The worker
  // clears pending commands, aborts the in-flight mode transition, and
  // resolves the promise it carries.
  bool estop_engaged_{false};
  std::shared_ptr<std::promise<Result>> estop_promise_;

  // Bounded, serializing command queue; RejectNew so the newest request
  // fails fast instead of displacing an older one.
  ThreadSafeQueue<Command> command_queue_;
  // Armed by execute_command after a successful set_mode; resolved by the
  // state-poll check. Worker-thread only.
  ModeTransition mode_transition_;

  // True while the latest velocity sent to the SDK was nonzero (worker
  // thread only). Used to detect the nonzero-to-zero transition that must
  // send one explicit zero move.
  bool last_sent_nonzero_{false};
  // True once the current connection's first sdk_linked snapshot armed the
  // driver readiness gate; cleared on every session teardown. Worker-thread
  // only.
  bool driver_ready_armed_{false};

  // Latest telemetry copies written by the SDK subscription callbacks.
  mutable std::mutex samples_mutex_;
  ImuSample latest_imu_;
  SportSample latest_sport_;
};

}  // namespace hypertron_ros2_bridge
