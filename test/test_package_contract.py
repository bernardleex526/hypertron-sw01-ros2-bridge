from pathlib import Path
import os
import re
import shutil
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]

# Documented vendor SDK 1.0.7 location on the commissioning host; override
# with ASTRALL_SDK_ROOT when running elsewhere.
DEFAULT_SDK_ROOT = "/home/lee/zkyd/xuanji/ASTRALL_SDK_1.0.7/C++"


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# Legacy public contract: artifacts and interfaces that stay valid for the
# direct driver. SSH/agent-deployment assertions are intentionally absent.
# ---------------------------------------------------------------------------

def test_required_artifacts_and_public_contract_exist() -> None:
    required = [
        "CMakeLists.txt",
        "package.xml",
        "REVIEW.md",
        "msg/RobotState.msg",
        "config/driver_config.yaml",
        "include/robot_controller.hpp",
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


def test_ros_graph_source_declares_required_interfaces() -> None:
    # driver_node.cpp is the single implementation source for the direct driver
    # ROS graph (any retired implementation has been removed).
    candidates = [ROOT / "src" / "driver_node.cpp"]
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in candidates
        if path.is_file()
    )
    assert source
    for token in (
        "geometry_msgs::msg::Twist",
        "sensor_msgs::msg::JointState",
        "std_msgs::msg::String",
        "std_srvs::srv::SetBool",
        "reject_joint_command",
    ):
        assert token in source


def test_documentation_keeps_ros_operating_contract_and_sdk_manual_facts() -> None:
    readme = read("README.md")
    for token in (
        "ROS2 Humble",
        "/cmd_vel",
        "/joint_commands",
        "/emergency_stop",
        "ros2 topic list",
        "ros2 service call",
    ):
        assert token in readme

    manual = read("SW01_MANUAL_NOTES.md")
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


# ---------------------------------------------------------------------------
# Direct-driver build contract.
# ---------------------------------------------------------------------------

def test_direct_driver_build_contract() -> None:
    cmake = read("CMakeLists.txt")
    package = read("package.xml")
    assert "ASTRALL_SDK_ROOT" in cmake
    assert "hypertron_driver_node" in cmake
    assert "src/direct_astrall_sdk.cpp" in cmake
    assert "libssh-dev" not in package
    assert "hypertron_bridge_agent" not in cmake
    assert "PkgConfig::LIBSSH" not in cmake


def test_direct_runtime_target_is_pure_and_reusable() -> None:
    """hypertron_direct_runtime compiles the driver runtime as a static
    library whose add_library definition lives outside the ROS block (pure
    configuration, no vendor include) and the runtime test links it instead
    of recompiling the source. The ROS node may link it inside the ROS block;
    only the definition must stay pure."""
    cmake = read("CMakeLists.txt")
    ros_block = "\n".join(bridge_block_lines(cmake))
    assert "add_library(hypertron_direct_runtime STATIC" in cmake
    assert "add_library(hypertron_direct_runtime STATIC" not in ros_block
    assert "src/direct_driver_runtime.cpp" in cmake
    assert cmake.count("hypertron_direct_runtime") >= 2


def test_vendor_include_is_unique_to_direct_adapter() -> None:
    """The vendor interface.h may only be included by the production
    adapter translation unit; every other source and header must stay
    vendor-free."""
    hits: list[str] = []
    for pattern in ("src/**/*.cpp", "include/**/*.hpp"):
        for path in sorted(ROOT.glob(pattern)):
            if not path.is_file():
                continue
            for line in path.read_text(encoding="utf-8",
                                       errors="ignore").splitlines():
                if re.search(r'#\s*include\s*[<"]interface\.h[>"]', line):
                    hits.append(path.relative_to(ROOT).as_posix())
    assert hits == ["src/direct_astrall_sdk.cpp"], hits


def test_package_xml_describes_direct_udp_driver() -> None:
    package = read("package.xml")
    match = re.search(r"<description>(.*?)</description>", package)
    assert match is not None
    assert "Direct UDP ROS 2 driver" in match.group(1)
    assert "ASTRALL SDK" in match.group(1)
    assert "libssh" not in package


def bridge_block_lines(cmake: str) -> list[str]:
    """Return the body of the if(BUILD_ROS2_BRIDGE) block, tolerant of
    indentation and blank lines."""
    lines = cmake.splitlines()
    depth = 0
    start = None
    for i, line in enumerate(lines):
        stripped = line.strip()
        if start is None:
            if stripped.startswith("if(BUILD_ROS2_BRIDGE"):
                start = i
                depth = 1
        else:
            if stripped.startswith("if("):
                depth += 1
            elif stripped == "endif()":
                depth -= 1
                if depth == 0:
                    return lines[start:i + 1]
    raise AssertionError("if(BUILD_ROS2_BRIDGE) block not found in CMakeLists.txt")


def test_sdk_discovery_is_scoped_to_production_bridge_block() -> None:
    block = "\n".join(bridge_block_lines(read("CMakeLists.txt")))
    assert "find_path(ASTRALL_INCLUDE_DIR" in block
    assert "find_library(ASTRALL_LIBRARY" in block
    assert '"${ASTRALL_SDK_ROOT}/include"' in block
    assert '"${ASTRALL_SDK_ROOT}/lib/linux/x86_64"' in block
    assert "hypertron_driver_node" in block
    assert 'target_link_libraries(hypertron_driver_node' in block
    assert '"${ASTRALL_LIBRARY}"' in block
    assert '"${ASTRALL_INCLUDE_DIR}"' in block
    assert "BUILD_RPATH" in block
    assert "INSTALL_RPATH" in block


# ---------------------------------------------------------------------------
# Executable contract: CMake sub-configurations without layout assumptions.
# ---------------------------------------------------------------------------

def run_sourced_cmake(
    build: Path,
    sdk_root: str,
    ros_setup: Path,
    build_driver: bool = False,
) -> subprocess.CompletedProcess[str]:
    """Configure (and optionally build) via bash with a sourced ROS setup.

    All paths travel through environment variables quoted in the script, so
    no user-controlled value is interpolated into the shell command line.
    """
    env = os.environ.copy()
    env["HTBR_ROS_SETUP"] = str(ros_setup)
    env["HTBR_SOURCE_DIR"] = str(ROOT)
    env["HTBR_BUILD_DIR"] = str(build)
    env["HTBR_SDK_ROOT"] = sdk_root
    script = (
        'source "$HTBR_ROS_SETUP" && '
        'cmake -S "$HTBR_SOURCE_DIR" -B "$HTBR_BUILD_DIR" '
        '-DASTRALL_SDK_ROOT="$HTBR_SDK_ROOT" -DBUILD_TESTING=OFF'
    )
    if build_driver:
        script += (
            ' && cmake --build "$HTBR_BUILD_DIR" '
            "--target hypertron_driver_node"
        )
    return subprocess.run(
        ["bash", "-c", script],
        env=env,
        capture_output=True,
        text=True,
    )


def require_sdk_and_ros() -> tuple[str, Path]:
    sdk_root = os.environ.get("ASTRALL_SDK_ROOT", DEFAULT_SDK_ROOT)
    if not (Path(sdk_root) / "include" / "interface.h").is_file():
        pytest.skip(f"ASTRALL SDK not present at {sdk_root}")
    ros_setup = next(Path("/opt/ros").glob("*/setup.bash"), None)
    if ros_setup is None:
        pytest.skip("no ROS setup.bash under /opt/ros")
    return sdk_root, ros_setup


def test_build_ros2_bridge_off_configures_without_sdk_root(tmp_path: Path) -> None:
    build = tmp_path / "build-off"
    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(ROOT),
            "-B",
            str(build),
            "-DBUILD_ROS2_BRIDGE=OFF",
            "-DBUILD_TESTING=OFF",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr


def test_invalid_sdk_root_fails_configuration(tmp_path: Path) -> None:
    build = tmp_path / "build-invalid"
    result = subprocess.run(
        [
            "cmake",
            "-S",
            str(ROOT),
            "-B",
            str(build),
            "-DASTRALL_SDK_ROOT=/nonexistent/astrall/sdk",
            "-DBUILD_TESTING=OFF",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "ASTRALL SDK x86_64 header/library not found" in result.stderr


def test_same_build_dir_reconfigure_with_invalid_root_fails(tmp_path: Path) -> None:
    # Cached find results must not let a stale valid SDK survive a
    # reconfigure that points ASTRALL_SDK_ROOT at a missing SDK.
    sdk_root, ros_setup = require_sdk_and_ros()
    build = tmp_path / "build-reuse"
    valid = run_sourced_cmake(build, sdk_root, ros_setup)
    assert valid.returncode == 0, valid.stdout + valid.stderr

    invalid_root = str(tmp_path / "missing-sdk")
    invalid = run_sourced_cmake(build, invalid_root, ros_setup)
    assert invalid.returncode != 0
    assert "ASTRALL SDK x86_64 header/library not found" in invalid.stderr


def test_valid_sdk_root_configures_direct_driver(tmp_path: Path) -> None:
    sdk_root, ros_setup = require_sdk_and_ros()
    readelf = shutil.which("readelf")
    if readelf is None:
        pytest.skip("readelf not available")
    build = tmp_path / "build-valid"
    result = run_sourced_cmake(build, sdk_root, ros_setup, build_driver=True)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "lib/linux/x86_64/libASTRALL_SDK.so" in result.stdout

    # The skeleton references no SDK symbols, yet the build contract requires
    # the vendor library to be a real DT_NEEDED dependency of the produced
    # executable (resolved via its SONAME), with a runpath to its directory.
    binary = build / "hypertron_driver_node"
    assert binary.is_file()
    dynamic = subprocess.run(
        [readelf, "-d", str(binary)],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    needed = [
        line.split("[", 1)[1].rstrip("]")
        for line in dynamic.splitlines()
        if "(NEEDED)" in line and "[" in line
    ]
    assert "libASTRALL_SDK.so.1" in needed
    sdk_lib_dir = str(Path(sdk_root) / "lib" / "linux" / "x86_64")
    rpath_lines = " ".join(
        line for line in dynamic.splitlines()
        if "RUNPATH" in line or "RPATH" in line
    )
    assert sdk_lib_dir in rpath_lines


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
