from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_required_artifacts_and_public_contract_exist() -> None:
    required = [
        "CMakeLists.txt",
        "package.xml",
        "msg/RobotState.msg",
        "config/bridge_config.yaml",
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
