// DirectAstrallSdk: the ONLY production translation unit allowed to include
// the vendor ASTRALL SDK interface.h. Every vendor callback validates its
// buffer, copies it into a project-owned value type, and only then invokes
// project callbacks. No vendor pointer ever leaves this file.
//
// Lifecycle, reentrancy, and threading contract:
//  - init() never requests control authority (non-actuating startup).
//  - init() installs the project callbacks and enters an accepting state
//    BEFORE calling the vendor, so a status callback delivered synchronously
//    by AstrallSdkInit reaches the project callback, and the linked/
//    authority values it delivers are retained after a successful init.
//    A failed init clears callbacks and all cached state.
//  - Every vendor entry point, including the snapshot getters, is
//    serialized through one lifecycle mutex, so the initialized check and
//    the vendor call are atomic with respect to deinit (no TOCTOU).
//  - Reentrancy: project callbacks may fire synchronously inside
//    AstrallSdkInit/AstrallSubscriptionData or asynchronously on vendor
//    threads. A thread is considered inside THIS adapter's callback
//    context when either its t_active_impl marker names this adapter (it
//    is executing one of its project callbacks) or its thread-local vendor
//    frame stack still contains this adapter (a vendor entry point is
//    still in progress — which also covers cross-adapter nesting, where an
//    OUTER adapter's synchronous callback re-enters this adapter).
//    Re-entering a public method from that context is rejected without
//    touching the vendor: snapshot() returns only cached state, every
//    other method returns a Running failure, and deinit() records a
//    deferred teardown. A recursive vendor call can therefore never happen
//    and no recursive mutex is needed.
//  - deinit() from inside a project callback never runs vendor teardown on
//    the vendor call stack: it only records the deferred flag. The full
//    teardown is completed by a NON-CALLBACK (runtime) thread — either a
//    later deinit() call or the tail checkpoint of a vendor entry point
//    invoked from outside callbacks. There is deliberately NO background
//    teardown thread; the adapter must never be destroyed on a vendor
//    callback thread (see the class contract in astrall_sdk.hpp).
//  - Full teardown: under the lifecycle mutex the state moves to
//    ShuttingDown and the gate closes (no new project callback can start),
//    then the vendor teardown runs, the lifecycle mutex is released, and
//    the teardown waits on the gate condition variable until every
//    in-flight project callback finished. Only then is the state moved to
//    Uninitialized. init() rejects any new generation while ShuttingDown,
//    so the in-flight wait can never absorb callbacks of a newer
//    generation.
//  - Cross-adapter deinit (adapter A's callback deinitializing adapter B)
//    is not reentrant for B and therefore takes B's normal waiting path.
//  - The destructor routes through deinit() and never skips the in-flight
//    wait: after deinit() returns, no project callback is executing or can
//    start. Destroying the adapter from a vendor callback thread is a
//    contract violation.
//  - Vendor callbacks hold only a weak_ptr to the adapter state and can
//    therefore never touch a destroyed adapter.
//  - Project callbacks are never invoked while a mutex is held.

#include <interface.h>

#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <memory>
#include <mutex>
#include <sstream>
#include <utility>
#include <vector>

#include "hypertron_ros2_bridge/astrall_sdk.hpp"

namespace hypertron_ros2_bridge {

namespace {

AstrallSubscribeFreq vendor_frequency(std::uint16_t frequency) {
  // SubscriptionFrequency values are numerically identical to the vendor
  // enum ordering; clamp unknown values to CLOSE for safety.
  switch (static_cast<SubscriptionFrequency>(frequency)) {
    case SubscriptionFrequency::Disabled:
      return ASTRALL_SUB_FREQ_CLOSE;
    case SubscriptionFrequency::Hz1:
      return ASTRALL_SUB_FREQ_1HZ;
    case SubscriptionFrequency::Hz25:
      return ASTRALL_SUB_FREQ_25HZ;
    case SubscriptionFrequency::Hz50:
      return ASTRALL_SUB_FREQ_50HZ;
    case SubscriptionFrequency::Hz125:
      return ASTRALL_SUB_FREQ_125HZ;
    case SubscriptionFrequency::Hz250:
      return ASTRALL_SUB_FREQ_250HZ;
  }
  return ASTRALL_SUB_FREQ_CLOSE;
}

Result map_vendor_result(std::uint16_t code) {
  if (code == ASTRALL_RES_SUCCESSED) {
    return Result::ok();
  }
  return Result::failure(code, describe_sdk_result_code(code));
}

Result not_initialized() {
  return Result::failure(kSdkResNotInit,
                         describe_sdk_result_code(kSdkResNotInit));
}

Result backend_unbound() {
  return Result::failure(kSdkResFailed, "vendor backend not bound");
}

Result reentrant_failure() {
  return Result::failure(kSdkResRunning,
                         "reentrant call from SDK callback not allowed");
}

}  // namespace

std::string describe_sdk_result_code(std::uint16_t code) {
  switch (code) {
    case kSdkResFailed:
      return "execution or request failed";
    case kSdkResTimeout:
      return "timeout";
    case kSdkResRunning:
      return "operation still running";
    case kSdkResSuccess:
      return "success";
    case kSdkResInvalidParam:
      return "invalid parameter";
    case kSdkResNotInit:
      return "SDK not initialized";
    case kSdkResRcNoRelease:
      return "remote controller has not released authority";
    case kSdkResBeenObtained:
      return "control authority already held by another device";
    case kSdkResWithoutAuth:
      return "no control authority";
    default: {
      std::ostringstream stream;
      stream << "unknown SDK result code 0x" << std::hex << code;
      return stream.str();
    }
  }
}

// The real binding to the official ASTRALL SDK functions. All vendor struct
// copying happens here, so every other translation unit stays vendor-free.
AstrallVendorApi make_real_astrall_vendor_api() {
  AstrallVendorApi api;
  api.init = [](RawDataCallback heartbeat_cb, RawDataCallback status_cb,
                std::uint32_t timeout_ms) -> std::uint16_t {
    AstrallConfig config;
    config.heartbeatCb = [heartbeat_cb](void* data, std::uint16_t len) {
      if (heartbeat_cb) {
        heartbeat_cb(data, len);
      }
    };
    config.sdkStatusCb = [status_cb](void* data, std::uint16_t len) {
      if (status_cb) {
        status_cb(data, len);
      }
    };
    return AstrallSdkInit(config, timeout_ms);
  };
  api.deinit = []() { AstrallSdkDeinit(); };
  api.heartbeat = [](std::uint32_t timeout_ms) {
    return AstrallHeartbeat(timeout_ms);
  };
  api.request_authority = [](AuthorityTarget target,
                             std::uint32_t timeout_ms) {
    const AstrallAuth auth = target == AuthorityTarget::Sdk
                                 ? ASTRALL_AUTH_SDK
                                 : ASTRALL_AUTH_JOYSTICK;
    return AstrallAuthControl(auth, timeout_ms);
  };
  api.subscribe = [](SubscriptionTopic topic, std::uint16_t frequency,
                     RawDataCallback data_cb, std::uint32_t timeout_ms) {
    AstrallSubscribeTopicId id;
    switch (topic) {
      case SubscriptionTopic::Imu:
        id = ASTRALL_SUB_TOPIC_ID_IMU;
        break;
      case SubscriptionTopic::Sport:
        id = ASTRALL_SUB_TOPIC_ID_SPORT;
        break;
      case SubscriptionTopic::Lidar:
        id = ASTRALL_SUB_TOPIC_ID_LIDAR;
        break;
    }
    return AstrallSubscriptionData(
        id, vendor_frequency(frequency),
        [data_cb](void* data, std::uint16_t len) {
          if (data_cb) {
            data_cb(data, len);
          }
        },
        timeout_ms);
  };
  api.set_mode = [](std::uint16_t mode, std::uint32_t timeout_ms) {
    return AstrallSportModeControl(static_cast<AstrallSportModeCmd>(mode),
                                   timeout_ms);
  };
  api.move = [](float vx, float vy, float vyaw, std::uint32_t timeout_ms) {
    return AstrallMove(vx, vy, vyaw, timeout_ms);
  };
  api.get_system_status = [](RawSystemStatus& out) -> std::uint16_t {
    AstrallSystemStatus status;
    const std::uint16_t code = AstrallGetSystemStatus(status);
    if (code == ASTRALL_RES_SUCCESSED) {
      out.sys_status = static_cast<std::uint8_t>(status.sysStatus);
      out.error_code = static_cast<std::uint32_t>(status.errorCode);
      out.warn_code = static_cast<std::uint32_t>(status.warnCode);
    }
    return code;
  };
  api.get_power_status = [](RawPowerStatus& out) -> std::uint16_t {
    AstrallPowerStatus power;
    const std::uint16_t code = AstrallGetPowerStatus(power);
    if (code == ASTRALL_RES_SUCCESSED) {
      out.soc = power.soc;
      out.temp = power.temp;
      out.voltage = power.voltage;
      out.cycle_count = power.cycleCount;
      out.charged = power.charged;
    }
    return code;
  };
  api.get_sport_status = []() { return AstrallGetSportStatus(); };
  return api;
}

namespace {

// Adapter state machine. Uninitialized rejects callbacks and vendor calls;
// Initializing accepts status callbacks during the vendor init call;
// Initialized accepts everything; ShuttingDown rejects callbacks and any
// new init generation while the teardown drain is running.
enum class State : std::uint8_t {
  Uninitialized,
  Initializing,
  Initialized,
  ShuttingDown,
};

// The adapter whose project callback the current thread is executing. Used
// to reject re-entrant public calls (never recursive vendor calls).
thread_local const detail::AstrallSdkImpl* t_active_impl = nullptr;

// Stack of adapter vendor entry points (one frame per Impl) the current
// thread is inside. Synchronous vendor callbacks (inside init/subscribe/...)
// run while the corresponding frame is on this stack. Checkpoints decide
// per-Impl, so a nested vendor frame of another adapter never delays this
// adapter's deferred teardown.
thread_local std::vector<const detail::AstrallSdkImpl*> t_vendor_frames;

bool in_vendor_frames(const detail::AstrallSdkImpl* impl) {
  return std::find(t_vendor_frames.begin(), t_vendor_frames.end(), impl) !=
         t_vendor_frames.end();
}

// True when the current thread is inside this adapter's callback context:
// either executing one of its project callbacks (t_active_impl) or still
// inside one of its vendor entry frames on the thread-local frame stack.
// The frame check covers cross-adapter nesting, where an OUTER adapter's
// synchronous callback re-enters this adapter while this adapter's vendor
// call is still on the stack: without it, the re-entry would try to take
// the lifecycle mutex of an in-progress vendor call and deadlock.
bool in_callback_context(const detail::AstrallSdkImpl* impl) {
  return t_active_impl == impl || in_vendor_frames(impl);
}

// RAII: mark the current thread as executing a project callback of `impl`.
struct CallbackScopeGuard {
  const detail::AstrallSdkImpl* previous;
  explicit CallbackScopeGuard(const detail::AstrallSdkImpl* impl)
      : previous(t_active_impl) {
    t_active_impl = impl;
  }
  ~CallbackScopeGuard() { t_active_impl = previous; }
};

// RAII: track that the current thread is inside an adapter vendor call of
// `impl`.
struct VendorCallScope {
  explicit VendorCallScope(const detail::AstrallSdkImpl* impl) {
    t_vendor_frames.push_back(impl);
  }
  ~VendorCallScope() { t_vendor_frames.pop_back(); }
};

}  // namespace

struct detail::AstrallSdkImpl {
  // Serializes every vendor entry point (init, deinit teardown, and all
  // calls including the snapshot getters).
  std::mutex lifecycle_mutex;
  AstrallVendorApi vendor;

  // Callback gate: accepting state, stored project callbacks, cached
  // telemetry, in-flight count, and the deferred-teardown request.
  std::mutex gate_mutex;
  std::condition_variable gate_cv;
  State state{State::Uninitialized};
  SdkCallbacks callbacks;
  ImuCallback imu_callback;
  SportCallback sport_callback;
  // Discard-semantics LIDAR subscription callback: a no-op installed through
  // the gate so deinit's in-flight drain is consistent (an asynchronous
  // discard delivery is waited out exactly like any other project callback).
  std::function<void()> lidar_callback;
  bool status_linked{false};
  bool status_authority{false};
  SportSample latest_sport;
  std::size_t in_flight{0};
  bool deferred_teardown{false};
  // Serializes the entire "ensure Uninitialized" deinit loop of
  // non-callback threads. Prevents concurrent deinit() calls from
  // interleaving teardowns (a second teardown publishing Uninitialized
  // between a first teardown's vendor deinit and its own publication
  // could stale-overwrite a generation initialized in between).
  std::mutex teardown_mutex;
#ifdef HYPERTRON_ENABLE_TEST_HOOKS
  // Test-only lifecycle observer; never compiled into production builds.
  std::function<void(TestEvent)> test_hook;
#endif
};

namespace {

// RAII: decrement the in-flight callback count and wake deinit() waiters.
struct InFlightGuard {
  std::shared_ptr<detail::AstrallSdkImpl> impl;
  ~InFlightGuard() {
    std::lock_guard<std::mutex> gate(impl->gate_mutex);
    if (--impl->in_flight == 0) {
      impl->gate_cv.notify_all();
    }
  }
};

#ifdef HYPERTRON_ENABLE_TEST_HOOKS
// Copies the test hook out of the gate critical section and invokes it
// outside any lock, so a test observer can safely call back into the
// adapter (or wait on its own primitives).
void emit_test_event(const std::shared_ptr<detail::AstrallSdkImpl>& impl,
                     TestEvent event) {
  std::function<void(TestEvent)> hook;
  {
    std::lock_guard<std::mutex> gate(impl->gate_mutex);
    hook = impl->test_hook;
  }
  if (hook) {
    hook(event);
  }
}
#endif

// Full teardown. The caller must not hold the lifecycle or gate mutex.
// Under the lifecycle mutex the state moves to ShuttingDown and the gate
// closes (no new project callback can start, and init() rejects a new
// generation), then the gate mutex is RELEASED before the vendor teardown
// runs (so a delivery racing for the gate can always finish and vendor
// deinit can never participate in a gate lock cycle), and finally the
// teardown waits for every in-flight project callback to finish. The
// terminal publication (state=Uninitialized) happens in ONE gate critical
// section, so no observer ever sees an intermediate state. Waiting never
// holds the lifecycle mutex, so a callback re-entering (or another thread
// calling) the SDK cannot deadlock. This function only runs on
// non-callback threads (deinit() or a vendor entry point tail checkpoint
// invoked from outside callbacks).
void teardown_impl(const std::shared_ptr<detail::AstrallSdkImpl>& impl) noexcept {
  bool need_wait = false;
  bool run_vendor_deinit = false;
  {
    std::lock_guard<std::mutex> lifecycle(impl->lifecycle_mutex);
    {
      std::lock_guard<std::mutex> gate(impl->gate_mutex);
      impl->deferred_teardown = false;
      const bool already_shutting_down = impl->state == State::ShuttingDown;
      if (impl->state != State::Uninitialized) {
        need_wait = true;
        impl->state = State::ShuttingDown;
        impl->callbacks = SdkCallbacks{};
        impl->imu_callback = {};
        impl->sport_callback = {};
        impl->lidar_callback = {};
        impl->status_linked = false;
        impl->status_authority = false;
        run_vendor_deinit =
            !already_shutting_down && impl->vendor.deinit != nullptr;
      } else if (impl->in_flight > 0) {
        // Failed-init drain: the state is already Uninitialized (the failed
        // init cleaned up) but an asynchronous project callback is still in
        // flight. Move to ShuttingDown so no new init generation can start
        // while the drain waits — its publication would otherwise
        // stale-overwrite that generation. vendor.deinit must NOT be called
        // (the failed init already cleaned up).
        need_wait = true;
        impl->state = State::ShuttingDown;
      }
      // A state that is already Uninitialized can still have in-flight
      // callbacks (e.g. a failed init with an async callback in flight);
      // those must be drained too, so the wait is never skipped.
      need_wait = need_wait || impl->in_flight > 0;
    }  // gate mutex released BEFORE the vendor teardown
#ifdef HYPERTRON_ENABLE_TEST_HOOKS
    if (need_wait) {
      // ShuttingDown is published (gate closed, new init rejected) before
      // the drain/vendor phase; tests wait on this event.
      emit_test_event(impl, TestEvent::ShuttingDownPublished);
    }
#endif
    if (run_vendor_deinit) {
      try {
        impl->vendor.deinit();
      } catch (...) {
        // The vendor API is nominally C++; keep teardown noexcept.
      }
    }
  }  // lifecycle mutex released BEFORE the in-flight wait
  if (need_wait) {
    std::unique_lock<std::mutex> gate(impl->gate_mutex);
    impl->gate_cv.wait(gate, [&] { return impl->in_flight == 0; });
  }
  // Atomic publication of the terminal generation under one gate critical
  // section; also runs on the need_wait==false path so every call leaves a
  // consistent Uninitialized state.
  {
    std::lock_guard<std::mutex> gate(impl->gate_mutex);
    impl->state = State::Uninitialized;
    impl->gate_cv.notify_all();
  }
#ifdef HYPERTRON_ENABLE_TEST_HOOKS
  emit_test_event(impl, TestEvent::TeardownCompleted);
#endif
}

// Deferred-teardown handling at the end of a delivery. When this Impl's
// vendor call is still on the current thread's stack (synchronous
// callback), the tail checkpoint of that vendor entry point completes the
// teardown. For asynchronous deliveries the deferred flag (set by deinit()
// inside the project callback) is simply KEPT: this adapter has no
// background teardown thread, and teardown must never run on a vendor
// callback thread — a non-callback thread completes it via deinit() or a
// later vendor entry point tail checkpoint.
void maybe_teardown_after_deliver(
    const std::shared_ptr<detail::AstrallSdkImpl>& impl) {
  if (in_vendor_frames(impl.get())) {
    return;  // synchronous: handled by the vendor entry point tail
  }
  // Asynchronous delivery: keep the deferred flag; nothing runs here.
  (void)impl;
}

// Invoke the project status callback under the gate. Returns without any
// effect when the adapter is not accepting callbacks. Never holds a lock
// while running the project callback.
void deliver_status(const std::weak_ptr<detail::AstrallSdkImpl>& weak,
                    bool linked, bool authority) {
  const std::shared_ptr<detail::AstrallSdkImpl> impl = weak.lock();
  if (!impl) {
    return;
  }
  std::function<void(bool, bool)> callback;
  {
    std::lock_guard<std::mutex> gate(impl->gate_mutex);
    if (impl->state == State::Uninitialized ||
        impl->state == State::ShuttingDown) {
      return;  // gate closed
    }
    impl->status_linked = linked;
    impl->status_authority = authority;
    callback = impl->callbacks.on_status;
    ++impl->in_flight;
  }
  {
    InFlightGuard guard{impl};
    if (callback) {
      CallbackScopeGuard scope{impl.get()};
      try {
        callback(linked, authority);
      } catch (...) {
        // A throwing project callback must not unwind into the vendor SDK.
      }
    }
  }
  maybe_teardown_after_deliver(impl);
}

void deliver_imu(const std::weak_ptr<detail::AstrallSdkImpl>& weak,
                 const ImuSample& sample) {
  const std::shared_ptr<detail::AstrallSdkImpl> impl = weak.lock();
  if (!impl) {
    return;
  }
  std::function<void(const ImuSample&)> callback;
  {
    std::lock_guard<std::mutex> gate(impl->gate_mutex);
    if (impl->state == State::Uninitialized ||
        impl->state == State::ShuttingDown) {
      return;
    }
    callback = impl->imu_callback;
    ++impl->in_flight;
  }
  {
    InFlightGuard guard{impl};
    if (callback) {
      CallbackScopeGuard scope{impl.get()};
      try {
        callback(sample);
      } catch (...) {
      }
    }
  }
  maybe_teardown_after_deliver(impl);
}

void deliver_sport(const std::weak_ptr<detail::AstrallSdkImpl>& weak,
                   const SportSample& sample) {
  const std::shared_ptr<detail::AstrallSdkImpl> impl = weak.lock();
  if (!impl) {
    return;
  }
  std::function<void(const SportSample&)> callback;
  {
    std::lock_guard<std::mutex> gate(impl->gate_mutex);
    if (impl->state == State::Uninitialized ||
        impl->state == State::ShuttingDown) {
      return;
    }
    impl->latest_sport = sample;
    callback = impl->sport_callback;
    ++impl->in_flight;
  }
  {
    InFlightGuard guard{impl};
    if (callback) {
      CallbackScopeGuard scope{impl.get()};
      try {
        callback(sample);
      } catch (...) {
      }
    }
  }
  maybe_teardown_after_deliver(impl);
}

void deliver_lidar_discard(const std::weak_ptr<detail::AstrallSdkImpl>& weak) {
  // Discard-semantics LIDAR subscription: enables the robot's UDP push
  // streams; no payload is meaningful, but the delivery still runs through
  // the same gate machinery (guard, in-flight count, reentrancy scope) so
  // deinit's in-flight drain and the teardown checkpoint behave exactly as
  // for every other project callback.
  const std::shared_ptr<detail::AstrallSdkImpl> impl = weak.lock();
  if (!impl) {
    return;
  }
  std::function<void()> callback;
  {
    std::lock_guard<std::mutex> gate(impl->gate_mutex);
    if (impl->state == State::Uninitialized ||
        impl->state == State::ShuttingDown) {
      return;
    }
    callback = impl->lidar_callback;
    ++impl->in_flight;
  }
  {
    InFlightGuard guard{impl};
    if (callback) {
      CallbackScopeGuard scope{impl.get()};
      try {
        callback();
      } catch (...) {
      }
    }
  }
  maybe_teardown_after_deliver(impl);
}

ImuSample map_imu(const AstrallImuData& vendor_data) {
  ImuSample sample;
  sample.timestamp = vendor_data.timestamp;
  sample.accelerometer = {vendor_data.accelerometer[0],
                          vendor_data.accelerometer[1],
                          vendor_data.accelerometer[2]};
  sample.gyroscope = {vendor_data.gyroscope[0], vendor_data.gyroscope[1],
                      vendor_data.gyroscope[2]};
  sample.quaternion = {vendor_data.quaternion[0], vendor_data.quaternion[1],
                       vendor_data.quaternion[2], vendor_data.quaternion[3]};
  sample.pitch = vendor_data.pitch;
  sample.roll = vendor_data.roll;
  sample.yaw = vendor_data.yaw;
  sample.odom_x = vendor_data.odomX;
  sample.odom_y = vendor_data.odomY;
  return sample;
}

SportSample map_sport(const AstrallSportData& vendor_data) {
  SportSample sample;
  sample.timestamp = vendor_data.timestamp;
  sample.wheel_speed = {vendor_data.wheelSpeed[0], vendor_data.wheelSpeed[1],
                        vendor_data.wheelSpeed[2], vendor_data.wheelSpeed[3]};
  return sample;
}

}  // namespace

DirectAstrallSdk::DirectAstrallSdk()
    : impl_(std::make_shared<detail::AstrallSdkImpl>()) {
  impl_->vendor = make_real_astrall_vendor_api();
}

DirectAstrallSdk::DirectAstrallSdk(AstrallVendorApi api)
    : impl_(std::make_shared<detail::AstrallSdkImpl>()) {
  impl_->vendor = std::move(api);
}

DirectAstrallSdk::~DirectAstrallSdk() { deinit(); }

#ifdef HYPERTRON_ENABLE_TEST_HOOKS
void DirectAstrallSdk::set_test_hook(std::function<void(TestEvent)> hook) {
  std::lock_guard<std::mutex> gate(impl_->gate_mutex);
  impl_->test_hook = std::move(hook);
}
#endif

bool DirectAstrallSdk::finish_with_deferred_teardown(
    std::unique_lock<std::mutex>& lifecycle) noexcept {
  // Per-Impl decision: only when THIS adapter's vendor frames have fully
  // left the current thread's stack. Nested vendor frames of other
  // adapters must not delay this teardown.
  if (in_vendor_frames(impl_.get())) {
    return false;
  }
  bool claimed = false;
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->deferred_teardown) {
      impl_->deferred_teardown = false;
      claimed = true;
    }
  }
  if (!claimed) {
    return false;
  }
  // Synchronous path: the vendor call that delivered the callback has
  // already returned, so this thread (the caller of the vendor entry
  // point) is no longer inside any vendor call stack. Running the full
  // teardown inline is safe — through the same serialized publisher
  // (teardown_mutex) used by deinit(), so at most one teardown publisher
  // exists at any time.
  lifecycle.unlock();
  ensure_uninitialized();
  return true;
}

Result DirectAstrallSdk::init(const SdkCallbacks& callbacks,
                              std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state == State::ShuttingDown) {
      return Result::failure(kSdkResFailed, "deinit in progress");
    }
    if (impl_->state != State::Uninitialized) {
      return Result::failure(kSdkResFailed, "SDK already initialized");
    }
    // Install the project callbacks and enter the accepting state BEFORE
    // the vendor call, so a status callback delivered synchronously by
    // AstrallSdkInit reaches the project callback and its caches.
    impl_->callbacks = callbacks;
    impl_->status_linked = false;
    impl_->status_authority = false;
    impl_->latest_sport = SportSample{};
    impl_->state = State::Initializing;
  }

  if (!impl_->vendor.init) {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    impl_->state = State::Uninitialized;
    impl_->callbacks = SdkCallbacks{};
    return backend_unbound();
  }

  const std::weak_ptr<detail::AstrallSdkImpl> weak = impl_;
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    code = impl_->vendor.init(
        // The vendor heartbeat callback carries no payload; nothing to
        // forward.
        [](const void*, std::uint16_t) {},
        [weak](const void* data, std::uint16_t len) {
          // Short or empty buffers are dropped before any copy: no state
          // update, no project callback.
          if (data == nullptr || len < sizeof(AstrallSdkStatus)) {
            return;
          }
          AstrallSdkStatus status{};
          std::memcpy(&status, data, sizeof(status));
          deliver_status(weak, status.link != 0U, status.ctrlAuthority != 0U);
        },
        timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  if (code != ASTRALL_RES_SUCCESSED) {
    // Failed init: clean every state and callback so nothing leaks past
    // this attempt.
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    impl_->state = State::Uninitialized;
    impl_->callbacks = SdkCallbacks{};
    impl_->status_linked = false;
    impl_->status_authority = false;
    return map_vendor_result(code);
  }

  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    impl_->state = State::Initialized;
    // Keep status caches from callbacks delivered during init: they are not
    // reset after a successful init.
  }
  return Result::ok();
}

void DirectAstrallSdk::deinit() noexcept {
  if (in_callback_context(impl_.get())) {
    // Called from this adapter's project callback (synchronous or
    // asynchronous): never run vendor teardown on the vendor call stack.
    // Only record the request; a non-callback (runtime) thread completes
    // the teardown via deinit() or a vendor entry point tail checkpoint.
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    impl_->deferred_teardown = true;
    return;
  }
  // Non-callback thread: run the full teardown inline. The whole "ensure
  // Uninitialized" loop is serialized by teardown_mutex so concurrent
  // deinit() calls can never interleave teardowns and stale-overwrite a
  // generation initialized in between. The loop re-checks the state after
  // every teardown so a new generation initialized in the meantime is torn
  // down too, and it only returns while observing Uninitialized with no
  // pending request AND no in-flight project callback (the latter also
  // covers callbacks still running after a failed init).
  ensure_uninitialized();
}

void DirectAstrallSdk::ensure_uninitialized() noexcept {
#ifdef HYPERTRON_ENABLE_TEST_HOOKS
  // Emitted at the entry, BEFORE taking teardown_mutex, so a caller that
  // is queued behind another teardown is observable too.
  emit_test_event(impl_, TestEvent::EnteredEnsureUninitialized);
#endif
  // Serialized by teardown_mutex: every publisher (deinit() and the vendor
  // entry tail checkpoints) goes through here, so at most one teardown
  // publisher exists at any time.
  std::lock_guard<std::mutex> teardown(impl_->teardown_mutex);
  for (;;) {
    std::unique_lock<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state == State::Uninitialized &&
        !impl_->deferred_teardown && impl_->in_flight == 0) {
      return;  // no active generation, no pending request, no in-flight cb
    }
    impl_->deferred_teardown = false;
    gate.unlock();
    teardown_impl(impl_);
  }
}

Result DirectAstrallSdk::heartbeat(std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return not_initialized();
    }
  }
  if (!impl_->vendor.heartbeat) {
    return backend_unbound();
  }
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    code = impl_->vendor.heartbeat(timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  return map_vendor_result(code);
}

Result DirectAstrallSdk::request_authority(bool sdk,
                                           std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return not_initialized();
    }
  }
  if (!impl_->vendor.request_authority) {
    return backend_unbound();
  }
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    code = impl_->vendor.request_authority(
        sdk ? AuthorityTarget::Sdk : AuthorityTarget::Joystick, timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  return map_vendor_result(code);
}

Result DirectAstrallSdk::subscribe_imu(SubscriptionFrequency frequency,
                                       ImuCallback callback,
                                       std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return not_initialized();
    }
    // Install the project callback BEFORE the vendor call so a first frame
    // delivered synchronously inside the subscription reaches it. The
    // previous callback is kept in `callback` for rollback on failure.
    std::swap(impl_->imu_callback, callback);
  }
  if (!impl_->vendor.subscribe) {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    std::swap(impl_->imu_callback, callback);  // rollback
    return backend_unbound();
  }
  const std::weak_ptr<detail::AstrallSdkImpl> weak = impl_;
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    code = impl_->vendor.subscribe(
        SubscriptionTopic::Imu, static_cast<std::uint16_t>(frequency),
        [weak](const void* data, std::uint16_t len) {
          if (data == nullptr || len < sizeof(AstrallImuData)) {
            return;
          }
          AstrallImuData vendor_data{};
          std::memcpy(&vendor_data, data, sizeof(vendor_data));
          deliver_imu(weak, map_imu(vendor_data));
        },
        timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  if (code != ASTRALL_RES_SUCCESSED) {
    // Roll back to the previous callback (or empty state).
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    std::swap(impl_->imu_callback, callback);
    return map_vendor_result(code);
  }
  return Result::ok();
}

Result DirectAstrallSdk::subscribe_sport(SubscriptionFrequency frequency,
                                         SportCallback callback,
                                         std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return not_initialized();
    }
    std::swap(impl_->sport_callback, callback);
  }
  if (!impl_->vendor.subscribe) {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    std::swap(impl_->sport_callback, callback);  // rollback
    return backend_unbound();
  }
  const std::weak_ptr<detail::AstrallSdkImpl> weak = impl_;
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    code = impl_->vendor.subscribe(
        SubscriptionTopic::Sport, static_cast<std::uint16_t>(frequency),
        [weak](const void* data, std::uint16_t len) {
          if (data == nullptr || len < sizeof(AstrallSportData)) {
            return;
          }
          AstrallSportData vendor_data{};
          std::memcpy(&vendor_data, data, sizeof(vendor_data));
          deliver_sport(weak, map_sport(vendor_data));
        },
        timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  if (code != ASTRALL_RES_SUCCESSED) {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    std::swap(impl_->sport_callback, callback);
    return map_vendor_result(code);
  }
  return Result::ok();
}

Result DirectAstrallSdk::subscribe_lidar(std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  // Local discard callback; the swap installs it into the gate and restores
  // it on rollback, mirroring subscribe_imu/subscribe_sport.
  std::function<void()> callback = [] {};
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return not_initialized();
    }
    // Install a discard callback (a no-op held for the gate's in-flight
    // drain) BEFORE the vendor call, mirroring subscribe_imu/subscribe_sport
    // so a synchronous delivery inside the subscription reaches the gate. The
    // previous callback is kept for rollback on failure.
    std::swap(impl_->lidar_callback, callback);
  }
  if (!impl_->vendor.subscribe) {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    std::swap(impl_->lidar_callback, callback);  // rollback
    return backend_unbound();
  }
  const std::weak_ptr<detail::AstrallSdkImpl> weak = impl_;
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    // Frequency is fixed to 1 Hz (ASTRALL_SUB_FREQ_1HZ, numerically 1): any
    // non-zero frequency opens the UDP 6100/6101 push streams and the discard
    // callback never interprets a payload, so a minimal non-zero rate is all
    // this contract guarantees. The vendor returns ASTRALL_RES_SUCCEESSED and
    // the streams flow to the node's own sockets.
    code = impl_->vendor.subscribe(
        SubscriptionTopic::Lidar, static_cast<std::uint16_t>(SubscriptionFrequency::Hz1),
        [weak](const void*, std::uint16_t) {
          deliver_lidar_discard(weak);
        },
        timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  if (code != ASTRALL_RES_SUCCESSED) {
    // Roll back to the previous callback (or empty state).
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    std::swap(impl_->lidar_callback, callback);
    return map_vendor_result(code);
  }
  return Result::ok();
}

Result DirectAstrallSdk::move(Velocity velocity, std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return not_initialized();
    }
  }
  if (!impl_->vendor.move) {
    return backend_unbound();
  }
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    code = impl_->vendor.move(velocity.vx, velocity.vy, velocity.vyaw,
                              timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  return map_vendor_result(code);
}

Result DirectAstrallSdk::set_mode(std::uint16_t mode,
                                  std::uint32_t timeout_ms) {
  if (in_callback_context(impl_.get())) {
    return reentrant_failure();
  }
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return not_initialized();
    }
  }
  if (!impl_->vendor.set_mode) {
    return backend_unbound();
  }
  std::uint16_t code = 0;
  {
    VendorCallScope vendor_scope{impl_.get()};
    code = impl_->vendor.set_mode(mode, timeout_ms);
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return Result::failure(kSdkResRunning,
                           "deinit requested from SDK callback");
  }
  return map_vendor_result(code);
}

SdkSnapshot DirectAstrallSdk::snapshot() {
  if (in_callback_context(impl_.get())) {
    // Re-entrant call from this adapter's project callback: return only
    // cached state; never call a vendor getter and never take the lifecycle
    // mutex (the vendor call that delivered us may still hold it).
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    SdkSnapshot cached;
    cached.sdk_linked = impl_->status_linked;
    cached.control_authority = impl_->status_authority;
    cached.wheel_speed = impl_->latest_sport.wheel_speed;
    return cached;
  }
  SdkSnapshot out;
  // The lifecycle mutex is held across the state check AND every vendor
  // getter, so the initialized check cannot race deinit() and the getters
  // never run concurrently with vendor teardown.
  std::unique_lock<std::mutex> lifecycle(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> gate(impl_->gate_mutex);
    if (impl_->state != State::Initialized) {
      return out;  // untouched defaults; no vendor call is made
    }
    out.sdk_linked = impl_->status_linked;
    out.control_authority = impl_->status_authority;
    out.wheel_speed = impl_->latest_sport.wheel_speed;
  }
  // Getter calls run with the lifecycle mutex held but WITHOUT the gate
  // mutex, so a concurrent vendor status callback can deliver freely.
  {
    VendorCallScope vendor_scope{impl_.get()};
    if (impl_->vendor.get_system_status) {
      RawSystemStatus system;
      if (impl_->vendor.get_system_status(system) == ASTRALL_RES_SUCCESSED) {
        out.system_status = system.sys_status;
        out.error_code = system.error_code;
        out.warning_code = system.warn_code;
      }
    }
    if (impl_->vendor.get_power_status) {
      RawPowerStatus power;
      if (impl_->vendor.get_power_status(power) == ASTRALL_RES_SUCCESSED) {
        out.battery_percentage = power.soc;
        out.battery_temperature = power.temp;
        out.battery_voltage = power.voltage;
        out.battery_cycle_count = power.cycle_count;
        out.charge_status = power.charged;
      }
    }
    if (impl_->vendor.get_sport_status) {
      // AstrallGetSportStatus returns either a sport status (0x0000..0xB1FF)
      // or a failure code (0x8000..); only status values are stored.
      const std::uint16_t sport = impl_->vendor.get_sport_status();
      if ((sport & 0xF000U) != 0x8000U) {
        out.sport_status = sport;
      }
    }
  }
  if (finish_with_deferred_teardown(lifecycle)) {
    return out;
  }
  return out;
}

}  // namespace hypertron_ros2_bridge
