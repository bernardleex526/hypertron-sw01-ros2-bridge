#include "hypertron_ros2_bridge/direct_driver_runtime.hpp"

#include <algorithm>
#include <atomic>
#include <exception>
#include <utility>

namespace hypertron_ros2_bridge {
namespace {

// True while the current thread is inside a RuntimeObserver callback
// dispatch. stop() consults it to detect a call made from inside an
// observer callback (which may be dispatched on a vendor SDK thread) and
// avoids joining the worker from there: the worker can be inside an
// in-flight SDK call that is waiting for exactly this callback to return.
thread_local bool t_in_observer_callback = false;

// RAII marker for the duration of one observer callback dispatch. Safe
// against a throwing observer: the flag is restored on unwind. Reentrant:
// an outer callback that synchronously dispatches an inner callback keeps
// the previous value, so the flag stays set for the whole outer frame and
// stop() cannot misjudge the outer callback as exited.
class ObserverCallbackGuard {
 public:
  ObserverCallbackGuard() : previous_(t_in_observer_callback) {
    t_in_observer_callback = true;
  }
  ~ObserverCallbackGuard() { t_in_observer_callback = previous_; }

 private:
  bool previous_;
};

// Runs an observer callback inside an ObserverCallbackGuard (so stop() can
// detect an in-observer dispatch on a vendor thread) AND a try/catch(...).
// A throwing observer must never escape into a vendor SDK entry point, the
// worker loop, or a caller: the exception is swallowed and the caller's error
// counter is incremented so the loss is observable. Never rethrows and never
// calls back into the observer.
template <typename Fn>
void dispatch_observer_safely(std::atomic<uint64_t>& errors, Fn&& fn) noexcept {
  ObserverCallbackGuard guard;
  try {
    fn();
  } catch (...) {
    errors.fetch_add(1, std::memory_order_relaxed);
  }
}

// Vendor sport status values treated as the stable outcome of each mode
// request (ASTRALL SDK 1.0.7 manual).
constexpr std::uint16_t kSportStatusDamping = 0xB101U;
constexpr std::uint16_t kSportStatusStand = 0xB102U;
constexpr std::uint16_t kSportStatusDown = 0xB103U;
constexpr std::uint16_t kSportStatusMove = 0xB104U;
constexpr std::uint16_t kSportStatusAutoCharge = 0xB107U;
constexpr std::uint16_t kSportStatusExitCharge = 0xB10BU;
constexpr std::uint16_t kSportStatusRecovery = 0xB1FFU;

// Stable sport status that confirms a requested mode; 0 means the mode code
// is not part of the documented mapping (unreachable for validated requests).
std::uint16_t stable_status_for(std::uint16_t mode) {
  switch (mode) {
    case 0xA101U:
      return kSportStatusDamping;
    case 0xA102U:
      return kSportStatusStand;
    case 0xA103U:
      return kSportStatusDown;
    case 0xA104U:
      return kSportStatusMove;
    case 0xA105U:
      return kSportStatusAutoCharge;
    case 0xA106U:
      return kSportStatusExitCharge;
    case 0xA1FFU:
      return kSportStatusRecovery;
    default:
      return 0U;
  }
}

// Maps a RobotController mode-request rejection onto a vendor result code so
// callers can distinguish "busy" from "bad input" without SDK semantics
// leaking into the controller.
Result map_mode_rejection(const ModeDecision& decision) {
  switch (decision.error) {
    case BridgeError::InvalidCommand:
      if (decision.reason.find("pending") != std::string::npos) {
        return Result::failure(kSdkResRunning, decision.reason);
      }
      return Result::failure(kSdkResInvalidParam, decision.reason);
    case BridgeError::SdkDisconnected:
      return Result::failure(kSdkResNotInit, decision.reason);
    case BridgeError::NoControlAuthority:
      return Result::failure(kSdkResWithoutAuth, decision.reason);
    default:
      return Result::failure(kSdkResFailed, decision.reason);
  }
}

}  // namespace

DirectDriverRuntime::DirectDriverRuntime(
    std::unique_ptr<IAstrallSdk> sdk,
    std::unique_ptr<INetworkPreflight> preflight, IMonotonicClock& clock,
    RuntimeConfig config, RuntimeObserver* observer)
    : sdk_(std::move(sdk)),
      preflight_(std::move(preflight)),
      clock_(clock),
      config_(std::move(config)),
      observer_(observer),
      controller_(ControllerConfig{config_.deadman_ms}, clock_),
      command_queue_(config_.queue_capacity, OverflowPolicy::RejectNew) {}

DirectDriverRuntime::~DirectDriverRuntime() {
  stop();
  // Deinitialize only after the worker joined so no SDK entry point runs
  // concurrently. Harmless when the worker already deinitialized.
  sdk_->deinit();
}

void DirectDriverRuntime::start() {
  // stop_mutex_ serializes every worker_ access (creation here, join in
  // stop()), so concurrent start()/stop() never race on the std::thread.
  std::lock_guard<std::mutex> stop_lock(stop_mutex_);
  std::lock_guard<std::mutex> lock(wake_mutex_);
  if (worker_.joinable() || stop_requested_) {
    return;  // already running, or the lifecycle is finished
  }
  worker_ = std::thread(&DirectDriverRuntime::worker_main, this);
}

void DirectDriverRuntime::stop() noexcept {
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    stop_requested_ = true;
  }
  wake_cv_.notify_all();
  // Never join from a vendor callback thread, and never take stop_mutex_
  // here: the worker may be blocked inside an SDK call that is waiting for
  // this very callback to return, while another thread could already hold
  // stop_mutex_ and be blocked joining that worker. Returning without a
  // join is safe because the worker observes stop_requested_ and exits on
  // its own; the destructor joins from the owning (non-callback) thread.
  if (t_in_observer_callback) {
    return;
  }
  // Serialize the joinable check and the join: concurrent stop() calls
  // from plain threads must join the worker exactly once and all return.
  std::lock_guard<std::mutex> lock(stop_mutex_);
  if (worker_.joinable() &&
      worker_.get_id() != std::this_thread::get_id()) {
    worker_.join();
  }
}

void DirectDriverRuntime::submit_velocity(const Velocity& velocity) {
  // Non-blocking: only the thread-safe controller gate runs on this thread;
  // the worker re-sends the accepted latest value at the motion refresh
  // period. A rejected command is dropped with a warning event.
  const VelocityDecision decision =
      controller_.accept_velocity({velocity.vx, velocity.vy, velocity.vyaw});
  if (!decision.accepted) {
    notify_event(kRuntimeEventWarning,
                 "velocity rejected: " + decision.reason);
  }
}

std::future<Result> DirectDriverRuntime::trigger_estop(bool engage) {
  if (!engage) {
    // Release the software latch only; the controller forces zero and no
    // velocity or mode is replayed. Resolved immediately.
    controller_.clear_estop();
    auto promise = std::make_shared<std::promise<Result>>();
    set_promise(promise,
                Result::ok("software emergency stop cleared"));
    return promise->get_future();
  }

  // Latch synchronously so no further velocity can be accepted from this
  // instant on, regardless of what the worker is doing.
  controller_.trigger_estop();
  auto promise = std::make_shared<std::promise<Result>>();
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    // A newer request supersedes an older one the worker has not consumed.
    if (estop_promise_ != nullptr) {
      set_promise(estop_promise_,
                  Result::failure(kSdkResFailed,
                                  "emergency stop request superseded"));
    }
    estop_engaged_ = true;
    estop_promise_ = promise;
    if (stop_requested_ || !connected_ || !sdk_linked_ ||
        link_drop_latched_) {
      // No session to process the worker-side mark: the latch stands and
      // the caller is answered immediately.
      estop_engaged_ = false;
      estop_promise_.reset();
      set_promise(promise,
                  Result::failure(kSdkResNotInit,
                                  "runtime is disconnected; emergency stop "
                                  "latched locally"));
      return promise->get_future();
    }
  }
  wake_cv_.notify_all();
  return promise->get_future();
}

ImuSample DirectDriverRuntime::latest_imu() const {
  std::lock_guard<std::mutex> lock(samples_mutex_);
  return latest_imu_;
}

SportSample DirectDriverRuntime::latest_sport() const {
  std::lock_guard<std::mutex> lock(samples_mutex_);
  return latest_sport_;
}

std::uint64_t DirectDriverRuntime::observer_error_count() const {
  return observer_errors_.load(std::memory_order_relaxed);
}

std::future<Result> DirectDriverRuntime::request_authority(bool sdk) {
  auto promise = std::make_shared<std::promise<Result>>();
  Command command{Command::Kind::RequestAuthority, sdk, 0U, promise};
  {
    // The session-end drain holds this same lock while failing pending
    // commands, so a command enqueued here can never be missed by it.
    std::lock_guard<std::mutex> lock(wake_mutex_);
    if (stop_requested_ || !connected_ || !sdk_linked_ ||
        link_drop_latched_) {
      set_promise(promise,
                  Result::failure(kSdkResNotInit,
                                  "runtime is disconnected; authority request "
                                  "not enqueued"));
      return promise->get_future();
    }
    if (!command_queue_.push(std::move(command))) {
      set_promise(promise,
                  Result::failure(kSdkResFailed, "command queue is full"));
      return promise->get_future();
    }
  }
  wake_cv_.notify_all();
  return promise->get_future();
}

std::future<Result> DirectDriverRuntime::request_mode(const std::string& name) {
  auto promise = std::make_shared<std::promise<Result>>();
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    if (stop_requested_ || !connected_ || !sdk_linked_ ||
        link_drop_latched_) {
      set_promise(promise,
                  Result::failure(kSdkResNotInit,
                                  "runtime is disconnected; mode request not "
                                  "enqueued"));
      return promise->get_future();
    }
  }

  const ModeDecision decision = controller_.request_mode(name);
  if (!decision.accepted) {
    set_promise(promise, map_mode_rejection(decision));
    return promise->get_future();
  }

  Command command{Command::Kind::RequestMode, false, decision.mode, promise};
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    // The session may have ended between the gate check and the enqueue;
    // undo the controller acceptance so motion is not left gated.
    if (stop_requested_ || !connected_ || !sdk_linked_ ||
        link_drop_latched_) {
      controller_.complete_mode_transition(false);
      set_promise(promise,
                  Result::failure(kSdkResNotInit,
                                  "runtime disconnected before the mode "
                                  "command was enqueued"));
      return promise->get_future();
    }
    if (!command_queue_.push(std::move(command))) {
      controller_.complete_mode_transition(false);
      set_promise(promise,
                  Result::failure(kSdkResFailed, "command queue is full"));
      return promise->get_future();
    }
  }
  wake_cv_.notify_all();
  return promise->get_future();
}

void DirectDriverRuntime::worker_main() {
  std::chrono::milliseconds backoff = config_.reconnect_initial_delay_ms;
  while (!stopping()) {
    const NetworkExpectation expectation{config_.interface,
                                         config_.host_address,
                                         config_.robot_address};
    const NetworkDecision decision = preflight_->check(expectation);
    if (!decision.ready) {
      notify_event(kRuntimeEventWarning,
                   "network preflight failed: " + decision.message);
      invalidate_session();
      sdk_->deinit();
      backoff = wait_backoff(backoff);
      continue;
    }

    // A status callback received from here on (including one delivered
    // synchronously inside init or during the subscriptions) belongs to
    // this connection attempt: the link-drop latch must not carry over
    // from the previous session, and a linked=false report must end this
    // session as soon as it starts.
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      link_drop_latched_ = false;
    }
    const Result init_result =
        sdk_->init(make_sdk_callbacks(), config_.init_timeout_ms);
    if (!init_result.success()) {
      notify_event(kRuntimeEventWarning,
                   "sdk init failed: " + init_result.message);
      invalidate_session();
      sdk_->deinit();
      backoff = wait_backoff(backoff);
      continue;
    }
    if (stopping()) {
      sdk_->deinit();
      return;
    }

    const Result subscribe_result = subscribe_telemetry();
    if (!subscribe_result.success()) {
      notify_event(kRuntimeEventWarning,
                   "sdk subscription failed: " + subscribe_result.message);
      invalidate_session();
      sdk_->deinit();
      backoff = wait_backoff(backoff);
      continue;
    }
    if (stopping()) {
      sdk_->deinit();
      return;
    }

    // Connected; a fresh backoff sequence starts at the next disconnect.
    // A link drop only ends this session when the status callback delivers
    // it while the session is live, never when sdk_linked_ still holds a
    // stale value from an earlier phase.
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      connected_ = true;
    }
    // The driver readiness gate is deliberately NOT armed here: it re-arms
    // only when the first snapshot of this connection reports an SDK link,
    // so stale state from the previous session can never reopen motion.
    backoff = config_.reconnect_initial_delay_ms;
      notify_event(kRuntimeEventInfo, "connected");
    run_connected_loop();

    // Session over: no further command may be accepted, and every queued or
    // in-flight command must be answered so no caller is left waiting on a
    // future that can never resolve.
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      connected_ = false;
      fail_all_pending_locked();
    }
    invalidate_session();
    sdk_->deinit();
    if (stopping()) {
      return;
    }
    notify_event(kRuntimeEventError, "connection lost; reconnecting");
    backoff = wait_backoff(backoff);
  }
}

void DirectDriverRuntime::run_connected_loop() {
  std::uint32_t consecutive_heartbeat_failures = 0;
  const IMonotonicClock::time_point session_start = clock_.now();
  IMonotonicClock::time_point next_heartbeat =
      session_start + config_.heartbeat_period;
  IMonotonicClock::time_point next_state_poll =
      session_start + config_.state_poll_period;
  IMonotonicClock::time_point next_motion_refresh =
      session_start + config_.motion_refresh_period;
  // A fresh session knows no commanded velocity, so no zero move is due.
  last_sent_velocity_ = {};
  last_sent_nonzero_ = false;

  while (true) {
    {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      IMonotonicClock::time_point deadline = std::min(
          std::min(next_heartbeat, next_state_poll), next_motion_refresh);
      // An armed mode transition must settle at its own deadline, not at
      // the next state poll: wake at the transition deadline when it is
      // sooner than every periodic deadline.
      if (mode_transition_.active && mode_transition_.deadline < deadline) {
        deadline = mode_transition_.deadline;
      }
      if (wake_cv_.wait_until(
              lock, deadline,
              [this] {
                return stop_requested_ || link_drop_latched_ ||
                       command_queue_.size() > 0U || estop_engaged_ ||
                       motion_refresh_due_ ||
                       (mode_transition_.active &&
                        clock_.now() >= mode_transition_.deadline);
              })) {
        if (stop_requested_ || link_drop_latched_) {
          const bool stop_was_requested = stop_requested_;
          // The actuation below takes wake_mutex_ itself, so the guard
          // must be released first.
          lock.unlock();
          if (stop_was_requested) {
            // Shutdown while the session is still live: send zero velocity
            // and damping mode before the session is torn down, but only
            // when the SDK link and control authority are present.
            actuate_stop_if_authorized();
          }
          return;  // shutdown requested or the SDK reported a link drop
        }
        // A command or an immediate motion-tick request arrived; fall
        // through and execute it before the periodic work.
      }
    }

    const IMonotonicClock::time_point now = clock_.now();
    // A status callback can latch a link drop between the wait above and
    // this point. Commands enqueued before the drop must never execute:
    // re-check and bail through the same teardown path as the predicate
    // exit above.
    {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      if (stop_requested_ || link_drop_latched_) {
        const bool stop_was_requested = stop_requested_;
        // The actuation below takes wake_mutex_ itself, so the guard must
        // be released first.
        lock.unlock();
        if (stop_was_requested) {
          actuate_stop_if_authorized();
        }
        return;  // shutdown requested or the SDK reported a link drop
      }
    }
    // The emergency stop takes priority over every queued command, so it is
    // processed before anything is dequeued.
    handle_estop_if_requested();
    const std::optional<Command> command = command_queue_.try_pop();
    if (command.has_value()) {
      execute_command(*command);
    }
    // A status callback may request an immediate motion tick (e.g. a lost
    // control authority must stop a dispatched velocity right away, not at
    // the next refresh deadline). The request is consumed here, before the
    // periodic work, and forces the tick to run even when the deadline has
    // not passed yet.
    bool motion_refresh_requested = false;
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      motion_refresh_requested = motion_refresh_due_;
      motion_refresh_due_ = false;
    }
    if (now >= next_heartbeat) {
      if (sdk_->heartbeat(config_.heartbeat_call_timeout_ms).success()) {
        consecutive_heartbeat_failures = 0;
      } else if (++consecutive_heartbeat_failures >=
                 std::max<std::uint32_t>(1U,
                                         config_.heartbeat_max_failures)) {
        notify_event(kRuntimeEventError,
                     "sdk heartbeat failed; ending the connection");
        return;
      }
      next_heartbeat = now + config_.heartbeat_period;
    }
    if (motion_refresh_requested || now >= next_motion_refresh) {
      // The controller returns the latest accepted velocity while it stays
      // within the command deadman and the safety gate; after a deadman
      // timeout or a gate change it returns a rejection, which means the
      // robot must be brought to zero.
      const VelocityDecision tick = controller_.velocity_for_tick();
      const Velocity target = tick.accepted
                                  ? Velocity{tick.value.vx, tick.value.vy,
                                             tick.value.vyaw}
                                  : Velocity{};
      const bool target_nonzero =
          target.vx != 0.0F || target.vy != 0.0F || target.vyaw != 0.0F;
      // Send when the tick is nonzero, or when it changed from nonzero to
      // zero (deadman expiry or gate loss must reach the SDK once). A
      // steady zero is not re-sent.
      if (target_nonzero || last_sent_nonzero_) {
        const Result move_result =
            sdk_->move(target, config_.sdk_call_timeout_ms);
        if (!move_result.success()) {
          // A failed move never stops the tick loop; the sent-state is kept
          // unchanged so the next tick retries with the current target
          // (including the zero that a deadman or gate loss demands).
          notify_event(kRuntimeEventWarning,
                       "velocity move rejected by the SDK: " +
                           move_result.message);
        } else {
          last_sent_velocity_ = target;
          last_sent_nonzero_ = target_nonzero;
        }
      }
      next_motion_refresh = now + config_.motion_refresh_period;
    }
    // Mode-transition deadline settlement: checked every iteration so a
    // timeout lands at the transition deadline even when the next state
    // poll is far away. Runs after the estop and command processing, so a
    // stop or a newer command always wins over an expiring transition.
    if (mode_transition_.active && now >= mode_transition_.deadline) {
      settle_mode_transition_timeout();
    }
    if (now >= next_state_poll) {
      const SdkSnapshot snapshot = sdk_->snapshot();
      {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        last_snapshot_sport_status_ = snapshot.sport_status;
        last_snapshot_error_code_ = snapshot.error_code;
        // Link and authority always come from the status-callback cache:
        // an older snapshot must never overwrite a newer link or authority
        // change with stale values. The snapshot contributes only the
        // sport status and error code. The controller update runs inside
        // the same critical section so a status callback can never
        // interleave between composing the status and applying it.
        controller_.update_robot_state(
            ControllerStatus{sdk_linked_, control_authority_,
                             snapshot.sport_status, snapshot.error_code});
      }
      // The driver readiness gate re-arms only on the first snapshot of
      // this connection that reports an SDK link; a fresh session never
      // inherits readiness from the session that just ended.
      if (snapshot.sdk_linked && !driver_ready_armed_) {
        driver_ready_armed_ = true;
        controller_.set_driver_ready(true);
      }
      check_mode_transition(snapshot, now);
      if (observer_ != nullptr) {
        dispatch_observer_safely(observer_errors_,
                                 [&] { observer_->on_state(snapshot); });
      }
      next_state_poll = now + config_.state_poll_period;
    }
  }
}

Result DirectDriverRuntime::subscribe_telemetry() {
  const Result imu = sdk_->subscribe_imu(
      config_.imu_freq,
      [this](const ImuSample& sample) { handle_imu(sample); },
      config_.sdk_call_timeout_ms);
  if (!imu.success()) {
    return imu;
  }
  const Result sport = sdk_->subscribe_sport(
      config_.sport_freq,
      [this](const SportSample& sample) { handle_sport(sample); },
      config_.sdk_call_timeout_ms);
  if (!sport.success()) {
    return sport;
  }
  // LIDAR discard subscription: opens the robot's UDP 6100/6101 push streams.
  // Reuses the same failure -> session reconnect path as imu/sport; every
  // reconnection subscribes again.
  if (config_.enable_lidar_stream) {
    const Result lidar = sdk_->subscribe_lidar(config_.sdk_call_timeout_ms);
    if (!lidar.success()) {
      return lidar;
    }
  }
  return Result::ok();
}

SdkCallbacks DirectDriverRuntime::make_sdk_callbacks() {
  SdkCallbacks callbacks;
  callbacks.on_status = [this](bool linked, bool control_authority) {
    handle_status(linked, control_authority);
  };
  return callbacks;
}

void DirectDriverRuntime::handle_status(bool linked, bool control_authority) {
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    sdk_linked_ = linked;
    control_authority_ = control_authority;
    // A linked=false report latches the end of this session even when a
    // later report in the same session re-links: the session that reported
    // a drop is over, period.
    if (!linked) {
      link_drop_latched_ = true;
    }
    // The safety gate consumes the report immediately (linked, authority,
    // and the latest polled sport status / error code), so a lost link or
    // lost authority closes the gate without waiting for the next state
    // poll. The controller update runs inside the same critical section as
    // the cache write so two concurrent status callbacks can never reorder
    // their gate effects: an older report cannot resurrect authority after
    // a newer report dropped it. The SDK entry points themselves are only
    // ever called by the worker; the motion tick that actually stops the
    // robot runs there.
    controller_.update_robot_state(
        ControllerStatus{linked, control_authority,
                         last_snapshot_sport_status_,
                         last_snapshot_error_code_});
    motion_refresh_due_ = true;
  }
  wake_cv_.notify_all();
  if (observer_ != nullptr) {
    dispatch_observer_safely(observer_errors_, [&] {
      observer_->on_status(linked, control_authority);
    });
  }
}

void DirectDriverRuntime::handle_imu(const ImuSample& sample) {
  {
    std::lock_guard<std::mutex> lock(samples_mutex_);
    latest_imu_ = sample;
  }
  if (observer_ != nullptr) {
    dispatch_observer_safely(observer_errors_, [&] {
      observer_->on_imu(sample);
    });
  }
}

void DirectDriverRuntime::handle_sport(const SportSample& sample) {
  {
    std::lock_guard<std::mutex> lock(samples_mutex_);
    latest_sport_ = sample;
  }
  if (observer_ != nullptr) {
    dispatch_observer_safely(observer_errors_, [&] {
      observer_->on_sport(sample);
    });
  }
}

void DirectDriverRuntime::notify_event(int severity, std::string message) {
  if (observer_ != nullptr) {
    dispatch_observer_safely(observer_errors_, [&] {
      observer_->on_event(severity, std::move(message));
    });
  }
}

bool DirectDriverRuntime::stopping() const {
  std::lock_guard<std::mutex> lock(wake_mutex_);
  return stop_requested_;
}

std::chrono::milliseconds DirectDriverRuntime::wait_backoff(
    std::chrono::milliseconds current) {
  std::unique_lock<std::mutex> lock(wake_mutex_);
  // Interruptible: stop() sets the flag and notifies, returning immediately.
  wake_cv_.wait_for(lock, current, [this] { return stop_requested_; });
  std::chrono::milliseconds next = current * 2;
  if (next > config_.reconnect_max_delay_ms) {
    next = config_.reconnect_max_delay_ms;
  }
  return std::max(next, current);
}

void DirectDriverRuntime::execute_command(const Command& command) {
  try {
    if (command.kind == Command::Kind::RequestAuthority) {
      const Result result = sdk_->request_authority(
          command.authority_sdk, config_.sdk_call_timeout_ms);
      set_promise(command.promise, result);
      return;
    }

    const Result set_result =
        sdk_->set_mode(command.mode, config_.sdk_call_timeout_ms);
    if (!set_result.success()) {
      controller_.complete_mode_transition(false);
      set_promise(command.promise,
                  Result::failure(set_result.code,
                                  "set_mode rejected: " + set_result.message));
      return;
    }
    const std::uint16_t stable = stable_status_for(command.mode);
    if (stable == 0U) {
      controller_.complete_mode_transition(false);
      set_promise(command.promise,
                  Result::failure(kSdkResInvalidParam,
                                  "no stable sport status is mapped for the "
                                  "requested mode"));
      return;
    }
    mode_transition_.active = true;
    mode_transition_.stable_status = stable;
    mode_transition_.deadline = clock_.now() + config_.mode_timeout_ms;
    mode_transition_.promise = command.promise;
  } catch (const std::exception& error) {
    // An in-flight transition must not be left dangling either: abort it and
    // answer both promises.
    if (mode_transition_.active) {
      const auto transition_promise = mode_transition_.promise;
      mode_transition_.active = false;
      set_promise(transition_promise,
                  Result::failure(kSdkResFailed,
                                  "mode transition aborted by a command "
                                  "execution exception"));
    }
    controller_.complete_mode_transition(false);
    set_promise(command.promise,
                Result::failure(kSdkResFailed,
                                std::string("command execution failed: ") +
                                    error.what()));
  } catch (...) {
    if (mode_transition_.active) {
      const auto transition_promise = mode_transition_.promise;
      mode_transition_.active = false;
      set_promise(transition_promise,
                  Result::failure(kSdkResFailed,
                                  "mode transition aborted by a command "
                                  "execution exception"));
    }
    controller_.complete_mode_transition(false);
    set_promise(command.promise,
                Result::failure(kSdkResFailed,
                                "command execution failed with an unknown "
                                "exception"));
  }
}

void DirectDriverRuntime::check_mode_transition(
    const SdkSnapshot& snapshot, IMonotonicClock::time_point now) {
  if (!mode_transition_.active) {
    return;
  }
  if (snapshot.sport_status == mode_transition_.stable_status) {
    const auto promise = mode_transition_.promise;
    mode_transition_.active = false;
    controller_.complete_mode_transition(true);
    set_promise(promise,
                Result::ok("mode transition reached the stable sport status"));
  } else if (now >= mode_transition_.deadline) {
    // Fallback: the loop's deadline wake normally settles the timeout
    // first; this covers a poll that observes the deadline before it.
    settle_mode_transition_timeout();
  }
}

void DirectDriverRuntime::settle_mode_transition_timeout() {
  const auto promise = mode_transition_.promise;
  mode_transition_.active = false;
  controller_.complete_mode_transition(false);
  set_promise(promise,
              Result::failure(kSdkResTimeout,
                              "mode transition timed out before reaching "
                              "the stable sport status"));
}

void DirectDriverRuntime::handle_estop_if_requested() {
  std::shared_ptr<std::promise<Result>> promise;
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    if (!estop_engaged_) {
      return;
    }
    estop_engaged_ = false;
    promise = std::move(estop_promise_);
    estop_promise_.reset();
  }

  // Clear every pending command and the in-flight mode transition so motion
  // cannot resume through a stale request once the latch is lifted.
  if (mode_transition_.active) {
    const auto transition_promise = mode_transition_.promise;
    mode_transition_.active = false;
    set_promise(transition_promise,
                Result::failure(kSdkResFailed,
                                "mode transition aborted by the emergency "
                                "stop"));
  }
  controller_.complete_mode_transition(false);
  std::optional<Command> command;
  while ((command = command_queue_.try_pop()).has_value()) {
    if (command->kind == Command::Kind::RequestMode) {
      // Undo the controller acceptance so motion is not left gated by a
      // transition that will never be confirmed.
      controller_.complete_mode_transition(false);
    }
    set_promise(command->promise,
                Result::failure(kSdkResFailed,
                                "command aborted by the emergency stop"));
  }

  bool linked = false;
  bool authority = false;
  bool link_dropped = false;
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    linked = sdk_linked_;
    authority = control_authority_;
    link_dropped = link_drop_latched_;
  }
  if (!linked || !authority || link_dropped) {
    set_promise(promise,
                Result::failure(kSdkResWithoutAuth,
                                "no SDK link or control authority; "
                                "emergency stop latched locally only"));
    return;
  }
  if (send_zero_and_damping()) {
    set_promise(promise,
                Result::ok("emergency stop executed: zero velocity and "
                           "damping mode"));
  } else {
    set_promise(promise,
                Result::failure(kSdkResFailed,
                                "emergency stop latched but the zero-velocity "
                                "or damping command failed"));
  }
}

bool DirectDriverRuntime::send_zero_and_damping() {
  const Result move_result =
      sdk_->move(Velocity{}, config_.sdk_call_timeout_ms);
  if (!move_result.success()) {
    notify_event(kRuntimeEventWarning,
                 "zero-velocity move failed: " + move_result.message);
  }
  const Result mode_result =
      sdk_->set_mode(0xA101U /* damping */, config_.sdk_call_timeout_ms);
  if (!mode_result.success()) {
    notify_event(kRuntimeEventWarning,
                 "damping mode command failed: " + mode_result.message);
  }
  last_sent_velocity_ = {};
  last_sent_nonzero_ = false;
  return move_result.success() && mode_result.success();
}

void DirectDriverRuntime::actuate_stop_if_authorized() {
  bool linked = false;
  bool authority = false;
  bool link_dropped = false;
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    linked = sdk_linked_;
    authority = control_authority_;
    link_dropped = link_drop_latched_;
  }
  if (!linked || !authority || link_dropped) {
    // Without the SDK link and control authority no actuating call is made;
    // the session is torn down as-is.
    return;
  }
  send_zero_and_damping();
}

void DirectDriverRuntime::fail_all_pending_locked() {
  // A pending emergency-stop request must not outlive the session: the
  // latch itself stays set in the controller, but the caller's future is
  // answered here instead of waiting for a session that may never come.
  if (estop_engaged_) {
    estop_engaged_ = false;
    if (estop_promise_ != nullptr) {
      set_promise(estop_promise_,
                  Result::failure(kSdkResNotInit,
                                  "connection lost during the emergency stop; "
                                  "the software latch remains set"));
      estop_promise_.reset();
    }
  }
  if (mode_transition_.active) {
    const auto promise = mode_transition_.promise;
    mode_transition_.active = false;
    set_promise(promise,
                Result::failure(kSdkResNotInit,
                                "connection lost during a mode transition"));
  }
  std::optional<Command> command;
  while ((command = command_queue_.try_pop()).has_value()) {
    if (command->kind == Command::Kind::RequestMode) {
      // The controller accepted the request before it was enqueued; undo it
      // so motion is not left gated after the session ends.
      controller_.complete_mode_transition(false);
    }
    set_promise(command->promise,
                Result::failure(kSdkResNotInit,
                                "runtime disconnected; command dropped"));
  }
}

void DirectDriverRuntime::invalidate_session() {
  driver_ready_armed_ = false;
  // Clear the runtime-side link/authority and snapshot caches first: a
  // reconnected session must see false/false for command acceptance, estop
  // and stop actuation until its own status callback reports otherwise -
  // the previous session's true/true must never survive the teardown.
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    sdk_linked_ = false;
    control_authority_ = false;
    last_snapshot_sport_status_ = 0;
    last_snapshot_error_code_ = 0;
  }
  // Clear the controller status second: without this, link/authority/sport
  // values from the ended session would survive into the next one and
  // could combine with a re-armed readiness gate to open motion on stale
  // state.
  controller_.update_robot_state({});
  controller_.invalidate_connection();
}

void DirectDriverRuntime::set_promise(
    const std::shared_ptr<std::promise<Result>>& promise,
    const Result& result) noexcept {
  try {
    promise->set_value(result);
  } catch (...) {
    // A teardown and a caller-side failure can race to answer the same
    // command; the first answer wins and the second is discarded.
  }
}

}  // namespace hypertron_ros2_bridge
