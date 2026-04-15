import os
from pathlib import Path
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "start_navigation_headless.sh"


def _write_file(path: Path, content: str = "#!/usr/bin/env bash\n"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _prepare_env(tmp_path: Path, *, websockets_available: bool = True):
    ros_setup = tmp_path / "opt/ros/humble/setup.bash"
    slam_workspace = tmp_path / "lightning-lm-livox"
    auto_workspace = tmp_path / "ros2_ws"
    fake_python = tmp_path / "bin/fake_python3"

    _write_file(ros_setup)
    _write_file(slam_workspace / "install/setup.bash")
    _write_file(slam_workspace / "config/default_livox.yaml", "dummy: true\n")
    _write_file(auto_workspace / "install/setup.bash")
    _write_file(
        fake_python,
        f"""#!/usr/bin/env bash
set -euo pipefail
if [[ "${{1:-}}" == "-c" ]]; then
  code="${{2:-}}"
  if [[ "$code" == "import yaml" ]]; then
    exit 0
  fi
  if [[ "$code" == "import websockets" ]]; then
    {"exit 0" if websockets_available else "exit 1"}
  fi
fi
exec python3 "$@"
""",
    )
    fake_python.chmod(0o755)

    env = os.environ.copy()
    env.update(
        {
            "ROS_SETUP": str(ros_setup),
            "SLAM_WORKSPACE": str(slam_workspace),
            "AUTO_WORKSPACE": str(auto_workspace),
            "ENABLE_CPU_ONLINE": "false",
            "PYTHON_BIN": str(fake_python),
        }
    )
    return env


def _run_script(tmp_path: Path, *args: str, websockets_available: bool = True):
    return subprocess.run(
        ["bash", str(SCRIPT_PATH), *args],
        capture_output=True,
        check=False,
        cwd=str(REPO_ROOT),
        env=_prepare_env(tmp_path, websockets_available=websockets_available),
        text=True,
    )


def test_headless_navigation_defaults_to_g1_executor_uri_and_exploration_enabled(tmp_path):
    result = _run_script(tmp_path, "--dry-run")

    assert result.returncode == 0
    assert "start_navigation_headless.sh" in result.stdout
    assert "ws://172.16.21.205:8100/ws/navigation/executor" in result.stdout
    assert "g1_auto_explore.launch.py use_rviz:=false" in result.stdout
    assert "enable_realsense_scan_bridge:=true" in result.stdout
    assert "enable_exploration:=true" in result.stdout
    assert "navigation_executor.py --server-uri ws://172.16.21.205:8100/ws/navigation/executor" in result.stdout


def test_headless_navigation_short_flags_toggle_rviz_realsense_and_exploration(tmp_path):
    result = _run_script(tmp_path, "-rse", "--dry-run")

    assert result.returncode == 0
    assert "g1_auto_explore.launch.py use_rviz:=true" in result.stdout
    assert "enable_realsense_scan_bridge:=false" in result.stdout
    assert "enable_exploration:=false" in result.stdout


def test_headless_navigation_short_server_uri_override_updates_executor_command(tmp_path):
    result = _run_script(
        tmp_path,
        "--dry-run",
        "-u",
        "ws://10.0.0.5:8100/ws/navigation/executor",
    )

    assert result.returncode == 0
    assert "navigation_executor.py --server-uri ws://10.0.0.5:8100/ws/navigation/executor" in result.stdout


def test_headless_navigation_fails_fast_when_websockets_module_is_missing(tmp_path):
    result = _run_script(tmp_path, "--dry-run", websockets_available=False)

    assert result.returncode == 1
    assert "Missing required Python module: websockets" in result.stderr
    assert "sudo apt install python3-websockets" in result.stderr
