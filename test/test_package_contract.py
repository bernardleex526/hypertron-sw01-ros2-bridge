from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def test_required_artifacts_and_public_contract_exist() -> None:
    required = [
        "CMakeLists.txt",
        "package.xml",
        "HTBR_PROTOCOL.md",
        "REVIEW.md",
        "msg/RobotState.msg",
        "config/bridge_config.yaml",
        "include/ssh_tunnel.hpp",
        "include/protocol_handler.hpp",
        "include/robot_controller.hpp",
        "include/data_receiver.hpp",
        "src/main.cpp",
    ]
    assert all((ROOT / path).is_file() for path in required)

    message = (ROOT / "msg/RobotState.msg").read_text(encoding="utf-8")
    for field in (
        "ssh_connected",
        "sdk_linked",
        "joint_interface_available",
        "odometry_scale_verified",
        "rejected_joint_commands",
    ):
        assert field in message

    config = (ROOT / "config/bridge_config.yaml").read_text(encoding="utf-8")
    for key in ("ssh:", "astrall:", "topics:", "safety:", "odometry:", "camera:"):
        assert key in config


def test_ros_bridge_source_declares_required_graph_interfaces() -> None:
    source = (ROOT / "src/bridge_node.cpp").read_text(encoding="utf-8")
    for token in (
        "geometry_msgs::msg::Twist",
        "sensor_msgs::msg::JointState",
        "std_msgs::msg::String",
        "std_srvs::srv::SetBool",
        "MessageType::Hello",
        "MessageType::HelloAck",
        "MessageType::CmdVelocity",
        "MessageType::CmdMode",
        "MessageType::CmdEstop",
        "reject_joint_command",
    ):
        assert token in source

    receiver = (ROOT / "src/data_receiver.cpp").read_text(encoding="utf-8")
    assert "camera_queue" in receiver
    assert "camera_worker" in receiver
    assert "pc_receive_time_ns" in receiver


def test_operator_documentation_covers_deployment_safety_and_sources() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    manual = (ROOT / "SW01_MANUAL_NOTES.md").read_text(encoding="utf-8")
    for token in (
        "ROS2 Humble",
        "agent-only",
        "ARM64",
        "known_hosts",
        "/cmd_vel",
        "/joint_commands",
        "/emergency_stop",
        "断线",
        "第 26 页",
        "ros2 topic list",
        "ros2 service call",
        "unitreerobotics/unitree_ros2",
        "DeepRoboticsLab/Lite3_ROS",
    ):
        assert token in readme
    for token in (
        "AstrallSdkInit",
        "AstrallHeartbeat",
        "AstrallMove",
        "UDP 6000",
        "UDP 6100",
        "UDP 6101",
        "AstrallImuData",
        "AstrallSportData",
    ):
        assert token in manual


def test_readme_contains_complete_public_commissioning_sop() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    required = (
        "## 实机前必要条件",
        "## SOP 0：安全与厂家资料准备",
        "## SOP 1：网络与 SSH 信任建立",
        "## SOP 2：PC 端 ROS2 编译",
        "## SOP 3：机器人端 ARM64 agent 部署",
        "## SOP 4：配置文件逐项设置",
        "## SOP 5：启动、状态门禁与首次低速运动",
        "## SOP 6：断线与故障恢复",
        "ROS2 指令到 ASTRALL 对照",
        "CMD_VELOCITY",
        "AstrallMove(vx, vy, vyaw)",
        "<ROBOT_IP>",
        "odometry_scale_verified=false",
        "30 分钟",
        "不能声称已通过真实 SW01 实机验证",
    )
    assert all(token in readme for token in required)


def test_public_sample_only_exposes_effective_configuration() -> None:
    config = (ROOT / "config/bridge_config.yaml").read_text(encoding="utf-8")
    for removed_noop in (
        "imu_frequency:",
        "sport_frequency:",
        "auto_prepare_motion:",
        "output_encoding:",
        "\n      queue_capacity: 2",
    ):
        assert removed_noop not in config

    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for fixed_behavior in (
        "IMU subscription: fixed at 125 Hz",
        "sport subscription: fixed at 50 Hz",
        "automatic motion preparation: disabled",
        "camera decode queue: fixed capacity 2",
        "camera output encoding: fixed BGR8",
    ):
        assert fixed_behavior in readme


def test_timeout_split_and_ssh_example_are_consistent() -> None:
    config = (ROOT / "config/bridge_config.yaml").read_text(encoding="utf-8")
    # 手册默认管理网段示例，README SOP 1 引用同一地址。
    assert 'host: "10.18.0.100"' in config
    # PC 稳态存活超时与 agent 应用心跳安全超时相互独立。
    assert "application_timeout_ms: 1500" in config  # ssh 段（PC）
    assert "application_timeout_ms: 500" in config  # safety 段（agent）
    assert "estop_ack_timeout_ms: 2000" in config
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert "HYPERTRON_SSH_PASSWORD" in readme
    assert "10.18.0.100" in readme
    assert "rmw_qos_profile_services_default" in readme


def test_readme_documents_lidar_cmd_vel_and_universal_slam_boundaries() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for token in (
        "不转发 LiDAR 点云",
        "与 universal_slam 组合的适配",
        "归一化无量纲",
        "不是 SI 单位",
        "不能直接",
        "m/s、rad/s",
        "/points_raw",
        "odometry_scale_verified=false",
    ):
        assert token in readme
    protocol = (ROOT / "HTBR_PROTOCOL.md").read_text(encoding="utf-8")
    assert "6100 点云不在 HTBR 内传输" in protocol


def test_readme_commands_assume_the_repository_is_the_package_root() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for required in (
        "<HYPERTRON_WS>/src/hypertron_ros2_bridge",
        "git clone <REPOSITORY_URL>",
        "cmake -S .",
        "python -m pytest test/test_package_contract.py -q",
        "Press Ctrl+C in the velocity publisher terminal",
        "libavcodec-dev libavutil-dev libswscale-dev",
        "PC -> agent",
        "agent -> PC",
    ):
        assert required in readme


def test_apache_license_is_complete_and_installed(tmp_path: Path) -> None:
    license_path = ROOT / "LICENSE"
    text = license_path.read_text(encoding="utf-8")
    assert text.startswith("Apache License\n                           Version 2.0")
    assert "END OF TERMS AND CONDITIONS" in text

    build = tmp_path / "build"
    install = tmp_path / "install"
    subprocess.run(
        [
            "cmake",
            "-S",
            str(ROOT),
            "-B",
            str(build),
            "-DBUILD_ROS2_BRIDGE=OFF",
            "-DBUILD_AGENT=OFF",
            "-DBUILD_TESTING=OFF",
        ],
        check=True,
    )
    subprocess.run(
        ["cmake", "--install", str(build), "--prefix", str(install)],
        check=True,
    )
    installed = install / "share" / "hypertron_ros2_bridge" / "LICENSE"
    assert installed.read_text(encoding="utf-8") == text
