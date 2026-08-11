#include "hypertron_ros2_bridge/data_receiver.hpp"

#ifdef HYPERTRON_WITH_ROS2

#include <algorithm>
#include <array>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hypertron_ros2_bridge/msg/robot_state.hpp"

#ifdef HYPERTRON_ENABLE_CAMERA
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}
#endif

namespace hypertron_ros2_bridge {
namespace {

builtin_interfaces::msg::Time time_message(std::uint64_t nanoseconds) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000ULL);
  stamp.nanosec =
      static_cast<std::uint32_t>(nanoseconds % 1000000000ULL);
  return stamp;
}

std::array<double, 36> unknown_pose_covariance() {
  std::array<double, 36> value{};
  value[0] = value[7] = value[14] = 0.05;
  value[21] = value[28] = 0.10;
  value[35] = 0.15;
  return value;
}

}  // namespace

struct DataReceiver::Impl {
  Impl(rclcpp::Node& node_value, DataReceiverConfig config_value)
      : node(node_value),
        config(std::move(config_value)),
        camera_state(config.camera_enabled) {
    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu = node.create_publisher<sensor_msgs::msg::Imu>(config.imu_topic,
                                                       sensor_qos);
    joints = node.create_publisher<sensor_msgs::msg::JointState>(
        config.joint_states_topic, sensor_qos);
    odometry = node.create_publisher<nav_msgs::msg::Odometry>(
        config.odom_topic, sensor_qos);
    robot_state = node.create_publisher<msg::RobotState>(
        config.robot_state_topic,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    image = node.create_publisher<sensor_msgs::msg::Image>(config.camera_topic,
                                                           sensor_qos);
#ifdef HYPERTRON_ENABLE_CAMERA
    if (config.camera_enabled) initialize_decoder();
#endif
  }

  ~Impl() {
#ifdef HYPERTRON_ENABLE_CAMERA
    if (parser != nullptr) av_parser_close(parser);
    if (codec_context != nullptr) avcodec_free_context(&codec_context);
    if (packet != nullptr) av_packet_free(&packet);
    if (frame != nullptr) av_frame_free(&frame);
    if (sws != nullptr) sws_freeContext(sws);
#endif
  }

  rclcpp::Node& node;
  DataReceiverConfig config;
  CameraIngestState camera_state;
  std::mutex state_mutex;
  ReceiverConnectionState connection;
  std::atomic<std::uint32_t> camera_error_count{0};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu;
  // The publisher is intentionally advertised but silent. ASTRALL 1.0.7 has
  // no joint-state source, so publishing zeros would be dangerous fabrication.
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry;
  rclcpp::Publisher<msg::RobotState>::SharedPtr robot_state;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image;

#ifdef HYPERTRON_ENABLE_CAMERA
  const AVCodec* codec{};
  AVCodecParserContext* parser{};
  AVCodecContext* codec_context{};
  AVPacket* packet{};
  AVFrame* frame{};
  SwsContext* sws{};

  void initialize_decoder() {
    codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    parser = av_parser_init(AV_CODEC_ID_H264);
    codec_context = codec == nullptr ? nullptr : avcodec_alloc_context3(codec);
    packet = av_packet_alloc();
    frame = av_frame_alloc();
    if (codec == nullptr || parser == nullptr || codec_context == nullptr ||
        packet == nullptr || frame == nullptr ||
        avcodec_open2(codec_context, codec, nullptr) < 0) {
      throw std::runtime_error("failed to initialize FFmpeg H.264 decoder");
    }
  }

  void publish_decoded_frame(std::uint64_t timestamp_ns) {
    if (frame->width <= 0 || frame->height <= 0) {
      throw MappingError("decoded camera frame has invalid dimensions");
    }
    sws = sws_getCachedContext(
        sws, frame->width, frame->height,
        static_cast<AVPixelFormat>(frame->format), frame->width, frame->height,
        AV_PIX_FMT_BGR24, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (sws == nullptr) throw MappingError("sws_getCachedContext failed");
    sensor_msgs::msg::Image message;
    message.header.stamp = time_message(timestamp_ns);
    message.header.frame_id = config.camera_frame;
    message.height = static_cast<std::uint32_t>(frame->height);
    message.width = static_cast<std::uint32_t>(frame->width);
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = message.width * 3U;
    message.data.resize(static_cast<std::size_t>(message.step) * message.height);
    std::uint8_t* output[]{message.data.data()};
    int strides[]{static_cast<int>(message.step)};
    const auto rows = sws_scale(sws, frame->data, frame->linesize, 0,
                                frame->height, output, strides);
    if (rows != frame->height) throw MappingError("sws_scale failed");
    image->publish(std::move(message));
  }

  void decode_camera(const CameraChunkPayload& payload) {
    const std::uint8_t* input = payload.data.data();
    int remaining = static_cast<int>(payload.data.size());
    while (remaining > 0) {
      std::uint8_t* packet_data{};
      int packet_size{};
      const auto consumed = av_parser_parse2(
          parser, codec_context, &packet_data, &packet_size, input, remaining,
          AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
      if (consumed < 0) throw MappingError("invalid H.264 parser input");
      input += consumed;
      remaining -= consumed;
      if (packet_size == 0) {
        if (consumed == 0) break;
        continue;
      }
      packet->data = packet_data;
      packet->size = packet_size;
      if (avcodec_send_packet(codec_context, packet) < 0) {
        throw MappingError("avcodec_send_packet failed");
      }
      while (true) {
        const auto result = avcodec_receive_frame(codec_context, frame);
        if (result == AVERROR(EAGAIN) || result == AVERROR_EOF) break;
        if (result < 0) throw MappingError("avcodec_receive_frame failed");
        publish_decoded_frame(payload.receive_time_ns);
        av_frame_unref(frame);
      }
    }
  }
#endif

  void publish_state_payload(const RobotStatePayload& payload,
                             std::uint64_t timestamp_ns) {
    ReceiverConnectionState local;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      local = connection;
    }
    msg::RobotState message;
    message.header.stamp = time_message(timestamp_ns);
    message.header.frame_id = config.base_frame;
    message.ssh_connected = local.ssh_connected;
    message.agent_connected = local.agent_connected;
    message.sdk_linked = payload.sdk_linked;
    message.control_authority = payload.control_authority;
    message.emergency_stop = payload.emergency_stop;
    message.joint_interface_available = false;
    message.camera_available = payload.camera_available;
    message.odometry_scale_verified = payload.odometry_scale_verified;
    message.system_status = payload.system_status;
    message.error_code = payload.error_code;
    message.warning_code = payload.warning_code;
    message.sport_status = payload.sport_status;
    message.battery_percentage = payload.battery_percentage;
    message.battery_temperature = payload.battery_temperature;
    message.battery_voltage = payload.battery_voltage;
    message.battery_cycle_count = payload.battery_cycle_count;
    message.charge_status = payload.charge_status;
    message.wheel_speed = payload.wheel_speed;
    message.protocol_rx_drops = local.protocol_rx_drops;
    message.rejected_joint_commands = local.rejected_joint_commands;
    message.last_error = local.last_error.empty() ? payload.last_error
                                                  : local.last_error;
    robot_state->publish(std::move(message));
  }
};

DataReceiver::DataReceiver(rclcpp::Node& node, DataReceiverConfig config)
    : impl_(std::make_unique<Impl>(node, std::move(config))) {}
DataReceiver::~DataReceiver() = default;

void DataReceiver::handle(const Frame& frame, std::uint64_t receive_time_ns) {
  try {
    if (frame.type == MessageType::Imu) {
      const auto sample =
          to_imu_sample(decode_imu(frame.payload), receive_time_ns, impl_->config);
      sensor_msgs::msg::Imu message;
      message.header.stamp = time_message(sample.timestamp_ns);
      message.header.frame_id = sample.frame_id;
      message.linear_acceleration.x = sample.linear_acceleration[0];
      message.linear_acceleration.y = sample.linear_acceleration[1];
      message.linear_acceleration.z = sample.linear_acceleration[2];
      message.angular_velocity.x = sample.angular_velocity[0];
      message.angular_velocity.y = sample.angular_velocity[1];
      message.angular_velocity.z = sample.angular_velocity[2];
      message.orientation.x = sample.orientation[0];
      message.orientation.y = sample.orientation[1];
      message.orientation.z = sample.orientation[2];
      message.orientation.w = sample.orientation[3];
      message.orientation_covariance = sample.orientation_covariance;
      message.angular_velocity_covariance =
          sample.angular_velocity_covariance;
      message.linear_acceleration_covariance =
          sample.linear_acceleration_covariance;
      impl_->imu->publish(std::move(message));
    } else if (frame.type == MessageType::Odometry) {
      const auto sample = to_odometry_sample(decode_odometry(frame.payload),
                                             receive_time_ns, impl_->config);
      nav_msgs::msg::Odometry message;
      message.header.stamp = time_message(sample.timestamp_ns);
      message.header.frame_id = sample.frame_id;
      message.child_frame_id = sample.child_frame_id;
      message.pose.pose.position.x = sample.position[0];
      message.pose.pose.position.y = sample.position[1];
      message.pose.pose.position.z = sample.position[2];
      message.pose.pose.orientation.x = sample.orientation[0];
      message.pose.pose.orientation.y = sample.orientation[1];
      message.pose.pose.orientation.z = sample.orientation[2];
      message.pose.pose.orientation.w = sample.orientation[3];
      message.pose.covariance = unknown_pose_covariance();
      impl_->odometry->publish(std::move(message));
    } else if (frame.type == MessageType::RobotState) {
      impl_->publish_state_payload(decode_robot_state(frame.payload),
                                   receive_time_ns);
    } else if (frame.type == MessageType::CameraH264) {
      const auto payload = decode_camera_chunk(frame.payload);
      if (!impl_->camera_state.accept(payload)) return;
#ifdef HYPERTRON_ENABLE_CAMERA
      impl_->decode_camera(payload);
#else
      impl_->camera_error_count.fetch_add(1);
#endif
    }
  } catch (const std::exception& error) {
    if (frame.type == MessageType::CameraH264) {
      impl_->camera_error_count.fetch_add(1);
    }
    std::lock_guard<std::mutex> lock(impl_->state_mutex);
    impl_->connection.last_error = error.what();
  }
}

void DataReceiver::set_connection_state(const ReceiverConnectionState& state) {
  std::lock_guard<std::mutex> lock(impl_->state_mutex);
  impl_->connection = state;
}

void DataReceiver::publish_disconnected_state() {
  RobotStatePayload state;
  state.odometry_scale_verified = impl_->config.odometry_scale_verified;
  impl_->publish_state_payload(state, static_cast<std::uint64_t>(
      impl_->node.get_clock()->now().nanoseconds()));
}

std::uint32_t DataReceiver::camera_errors() const noexcept {
  return impl_->camera_error_count.load();
}

}  // namespace hypertron_ros2_bridge

#endif  // HYPERTRON_WITH_ROS2
