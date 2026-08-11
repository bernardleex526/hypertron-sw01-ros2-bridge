#include "hypertron_ros2_bridge/astrall_sdk_adapter.hpp"

#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <stdexcept>

namespace hypertron_ros2_bridge {

struct PosixByteStream::Impl {
  int input_fd{-1};
  int output_fd{-1};
  int wake_fd{-1};
  bool own_data_fds{};
  std::atomic_bool closed{false};
};

PosixByteStream::PosixByteStream(int input_fd, int output_fd,
                                 bool own_data_fds)
    : impl_(std::make_unique<Impl>()) {
  if (input_fd < 0 || output_fd < 0) {
    throw std::invalid_argument("POSIX stream file descriptors must be valid");
  }
  impl_->input_fd = input_fd;
  impl_->output_fd = output_fd;
  impl_->own_data_fds = own_data_fds;
  impl_->wake_fd = ::eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  if (impl_->wake_fd < 0) {
    throw std::runtime_error("eventfd() failed for agent stream wakeup");
  }
}

PosixByteStream::~PosixByteStream() {
  close();
  if (impl_->own_data_fds) {
    ::close(impl_->input_fd);
    if (impl_->output_fd != impl_->input_fd) ::close(impl_->output_fd);
  }
  ::close(impl_->wake_fd);
}

std::optional<std::vector<std::uint8_t>> PosixByteStream::read_some(
    std::chrono::milliseconds timeout) {
  if (impl_->closed.load()) return std::nullopt;
  pollfd descriptors[2]{{impl_->input_fd, POLLIN, 0},
                        {impl_->wake_fd, POLLIN, 0}};
  const auto ready =
      ::poll(descriptors, 2, static_cast<int>(timeout.count()));
  if (ready < 0) {
    if (errno == EINTR) return std::vector<std::uint8_t>{};
    return std::nullopt;
  }
  if (ready == 0) return std::vector<std::uint8_t>{};
  if ((descriptors[1].revents & POLLIN) != 0) {
    std::uint64_t ignored{};
    const auto wake_count = ::read(impl_->wake_fd, &ignored, sizeof(ignored));
    if (wake_count < 0 && errno != EAGAIN && errno != EINTR) {
      return std::nullopt;
    }
    return impl_->closed.load()
               ? std::optional<std::vector<std::uint8_t>>{}
               : std::optional<std::vector<std::uint8_t>>(
                     std::vector<std::uint8_t>{});
  }
  if ((descriptors[0].revents & (POLLERR | POLLNVAL)) != 0 ||
      ((descriptors[0].revents & POLLHUP) != 0 &&
       (descriptors[0].revents & POLLIN) == 0)) {
    return std::nullopt;
  }
  if ((descriptors[0].revents & POLLIN) == 0) {
    return std::vector<std::uint8_t>{};
  }
  std::vector<std::uint8_t> bytes(64U * 1024U);
  const auto count = ::read(impl_->input_fd, bytes.data(), bytes.size());
  if (count < 0 && errno == EINTR) return std::vector<std::uint8_t>{};
  if (count <= 0) return std::nullopt;
  bytes.resize(static_cast<std::size_t>(count));
  return bytes;
}

bool PosixByteStream::write_all(const std::vector<std::uint8_t>& bytes) {
  if (impl_->closed.load()) return false;
  std::size_t offset{};
  while (offset < bytes.size()) {
    const auto count = ::write(impl_->output_fd, bytes.data() + offset,
                               bytes.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    offset += static_cast<std::size_t>(count);
  }
  return true;
}

void PosixByteStream::close() noexcept {
  if (impl_->closed.exchange(true)) return;
  const std::uint64_t wake = 1;
  const auto wake_count = ::write(impl_->wake_fd, &wake, sizeof(wake));
  if (wake_count < 0) return;
}

}  // namespace hypertron_ros2_bridge
