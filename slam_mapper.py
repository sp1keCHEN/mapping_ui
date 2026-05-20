#!/usr/bin/env python3
"""Interactive mapping script for ROS2-based 3D pointcloud + grid map creation."""

import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

# ---------------------------------------------------------------------------
# Path constants
# ---------------------------------------------------------------------------
HOME = Path.home()
ALGOR_WS = HOME / "Workspace" / "algor_ws" / "src"
ALGOR_WS_ROOT = ALGOR_WS.parent
FASTER_SLAM = ALGOR_WS / "faster_slam"
PGO_OUTPUT = FASTER_SLAM / "data" / "PGO_output"
PRIOR_DIR = FASTER_SLAM / "prior"
GRIDMAPPER_OUTPUT = ALGOR_WS / "gridmapper" / "data" / "Output"
MAPS_DIR = ALGOR_WS / "multi_map_nav_ros2" / "maps"

# ROS2 launch files
LIVOX_LAUNCH = "livox_ros_driver2 msg_multi_MID360_launch.py"
NAV_BRIDGE_LAUNCH = "nav_bridge nav_bridge.launch.py"
SLAM_PGO_LAUNCH = "faster_lio slam.launch.py pgo:=true rviz:=true"
GRIDMAPPER_LAUNCH = "gridmapper global.launch.py rviz:=true"

# Topics
LIVOX_TOPIC = "/livox/lidar"
IMU_TOPIC = "/imu/data"


# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------
def _ros2_env() -> dict:
    """Return an environment dict with ROS2 + colcon workspace sourced."""
    env = os.environ.copy()
    # Source setup.bash to set ROS_PACKAGE_PATH, etc.
    setup_bash = ALGOR_WS_ROOT / "install" / "setup.bash"
    if setup_bash.exists():
        install_prefix = str(ALGOR_WS_ROOT / "install")
        # Colcon setup.bash prepends to these variables
        for var, suffix in (
            ("ROS_PACKAGE_PATH", f"{install_prefix}/share"),
            ("PYTHONPATH", f"{install_prefix}/lib/python3.10/site-packages:{install_prefix}/lib/python3.12/site-packages"),
        ):
            if var in env:
                env[var] = f"{suffix}:{env[var]}"
            else:
                env[var] = suffix
        # Add colcon bin to PATH
        bin_dir = install_prefix
        if "PATH" in env:
            env["PATH"] = f"{bin_dir}:{env['PATH']}"
    return env


ROS2_ENV = _ros2_env()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def run(cmd: str, check: bool = True) -> subprocess.CompletedProcess:
    """Run a short-lived shell command and wait for it to finish."""
    print(f"[RUN] {cmd}")
    return subprocess.run(cmd, shell=True, check=check, env=ROS2_ENV)


def launch(cmd: str) -> subprocess.Popen:
    """Launch a long-running shell command in the background."""
    print(f"[LAUNCH] {cmd}")
    return subprocess.Popen(cmd, shell=True, env=ROS2_ENV)


def wait_for_node(node_name: str, timeout: int = 30) -> bool:
    """Wait until a ROS2 node appears in `ros2 node list`."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                "ros2 node list", shell=True, env=ROS2_ENV,
                capture_output=True, text=True, timeout=5,
            )
            if node_name in result.stdout:
                return True
        except Exception:
            pass
        time.sleep(1)
    return False


def wait_for_topic(topic: str, timeout: int = 20) -> bool:
    """Wait until a topic has at least one active publisher."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                f"ros2 topic info {topic} --no-arr",
                shell=True, env=ROS2_ENV,
                capture_output=True, text=True, timeout=5,
            )
            for line in result.stdout.splitlines():
                if "Publisher count:" in line:
                    count = int(line.split(":")[1].strip())
                    return count > 0
        except Exception:
            pass
        time.sleep(1)
    return False


def check_topic_hz(topic: str, timeout: int = 10) -> float:
    """Return the average Hz of a topic, or 0.0 if unavailable."""
    try:
        result = subprocess.run(
            f"ros2 topic hz {topic} --window 10 --timeout {timeout}",
            shell=True, env=ROS2_ENV,
            capture_output=True, text=True, timeout=timeout + 5,
        )
        for line in result.stdout.splitlines():
            if "average rate:" in line:
                return float(line.split("average rate:")[1].strip().split()[0])
    except Exception:
        pass
    return 0.0


def pause(msg: str = "") -> None:
    """Pause execution until the user presses Enter."""
    separator = "\n" + "-" * 60
    prompt_text = ""
    if msg:
        prompt_text = f"\n{msg}\n"
    input(f"{separator}{prompt_text}Press Enter to continue ...{separator}\n")


def confirm(prompt: str, default: bool = True) -> bool:
    """Ask a yes/no question. Return True for yes."""
    suffix = "[Y/n]" if default else "[y/N]"
    answer = input(f"{prompt} {suffix} ").strip().lower()
    if not answer:
        return default
    return answer in ("y", "yes")


def kill_process(p: subprocess.Popen) -> None:
    """Gracefully terminate a subprocess tree."""
    print(f"[STOP] Terminating process group {p.pid} ...")
    p.terminate()
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()
        p.wait()


# Ctrl+C handler
_RUNNING_PROCESSES: list = []


def _signal_handler(signum, frame):
    for p in _RUNNING_PROCESSES:
        if p.poll() is None:
            kill_process(p)
    print("\n[EXIT] Interrupted by user.")
    sys.exit(130)


signal.signal(signal.SIGINT, _signal_handler)


# ---------------------------------------------------------------------------
# Step 1 - Sensor data acquisition
# ---------------------------------------------------------------------------
def step1_sensor_setup():
    print("\n" + "=" * 60)
    print("Step 1: Sensor data acquisition")
    print("=" * 60)

    # 1. Start Livox lidar
    print("\n[1/3] Starting Livox lidar ...")
    livox_proc = launch(f"ros2 launch {LIVOX_LAUNCH}")
    _RUNNING_PROCESSES.append(livox_proc)

    if not wait_for_topic(LIVOX_TOPIC, timeout=20):
        print(f"[WARN] No publisher detected on {LIVOX_TOPIC} after 20s.")
    else:
        hz = check_topic_hz(LIVOX_TOPIC)
        print(f"[OK] {LIVOX_TOPIC} active (avg {hz:.1f} Hz)")

    # 2. Start nav_bridge for IMU
    print("\n[2/3] Starting nav_bridge for IMU data ...")
    nav_proc = launch(f"ros2 launch {NAV_BRIDGE_LAUNCH}")
    _RUNNING_PROCESSES.append(nav_proc)

    if not wait_for_topic(IMU_TOPIC, timeout=20):
        print(f"[WARN] No publisher detected on {IMU_TOPIC} after 20s.")
    else:
        hz = check_topic_hz(IMU_TOPIC)
        print(f"[OK] {IMU_TOPIC} active (avg {hz:.1f} Hz)")

    # 3. Release control
    print("\n[3/3] Releasing control to remote ...")
    if confirm("Call /nav_bridge_node/release_control ?", default=True):
        run(
            "ros2 service call /nav_bridge_node/release_control std_srvs/srv/Trigger"
        )
        print("[OK] Control released.")
    else:
        print("[SKIP] Control release skipped.")

    pause("Step 1 complete. Lidar and IMU are running. Verify topics, then proceed.")
    return livox_proc, nav_proc


# ---------------------------------------------------------------------------
# Step 2 - 3D pointcloud map (PGO)
# ---------------------------------------------------------------------------
def step2_pointcloud_slam():
    """Return (map_name, bag_dir)."""
    print("\n" + "=" * 60)
    print("Step 2: 3D pointcloud map construction (PGO)")
    print("=" * 60)

    # Ask for map name early
    now = datetime.now().strftime("%y%m%d_%H%M")
    default_name = input(f"\nMap name [default: sensor_{now}]? ").strip()
    if not default_name:
        default_name = f"sensor_{now}"
    map_name = default_name

    # 1. Stand up the dog
    print("\n[1/5] Please use the remote to stand up the robot.")
    pause()

    # 2. Start SLAM with PGO
    print("\n[2/5] Starting SLAM with PGO + Rviz ...")
    slam_proc = launch(f"ros2 launch {SLAM_PGO_LAUNCH}")
    _RUNNING_PROCESSES.append(slam_proc)

    if not wait_for_node("/faster_lio", timeout=30):
        print("[WARN] /faster_lio node not detected after 30s.")
    else:
        print("[OK] /faster_lio node is running.")

    # 3. Record bag
    bag_name = f"{map_name}_sensor"
    print(f"\n[3/5] Recording bag '{bag_name}' ...")
    bag_proc = launch(
        f"ros2 bag record -o {bag_name} {LIVOX_TOPIC} {IMU_TOPIC}",
    )
    _RUNNING_PROCESSES.append(bag_proc)
    print(f"[OK] Recording to {bag_name}/")

    # 4. Navigation phase
    print("\n[4/5] Navigation phase")
    print("  - Drive the robot through the scene with the remote.")
    print("  - Watch Rviz for loop closures and map quality.")
    print("  - Keep dynamic objects out of the robot's frontal view.")
    pause("Press Enter when mapping is complete.")

    # Stop bag recording
    print("\n[STOP] Stopping bag recording ...")
    kill_process(bag_proc)
    _RUNNING_PROCESSES.remove(bag_proc)
    print("[OK] Bag recording stopped.")

    # 5. Collect results
    print("\n[5/5] Collecting mapping results ...")

    # Find the actual bag directory (ROS2 appends a timestamp suffix)
    bag_dir = None
    for p in sorted(Path(".").glob(f"{bag_name}_*")):
        if p.is_dir():
            bag_dir = p
            break
    if bag_dir:
        print(f"[OK] Bag saved at ./{bag_dir}")
    else:
        print(f"[WARN] Bag directory for '{bag_name}' not found.")

    # Copy PGO output to prior/
    pgo_pcd = PGO_OUTPUT / "PGO.pcd"
    pgo_kf = PGO_OUTPUT / "keyframes"

    if not pgo_pcd.exists():
        print(f"[ERROR] {pgo_pcd} not found. Skipping prior copy.")
    elif not pgo_kf.is_dir():
        print(f"[ERROR] {pgo_kf} not found. Skipping prior copy.")
    else:
        dest = PRIOR_DIR / map_name
        dest.mkdir(parents=True, exist_ok=True)

        print(f"  Copying PGO.pcd -> {dest}/")
        shutil.copy2(pgo_pcd, dest / "PGO.pcd")

        print(f"  Copying keyframes/ -> {dest}/keyframes/")
        if (dest / "keyframes").exists():
            shutil.rmtree(dest / "keyframes")
        shutil.copytree(pgo_kf, dest / "keyframes")

        print(f"[OK] Prior saved to {dest}/")

    # Stop SLAM
    print("\n[STOP] Stopping SLAM node ...")
    kill_process(slam_proc)
    _RUNNING_PROCESSES.remove(slam_proc)

    pause("Step 2 complete.")
    return map_name, str(bag_dir) if bag_dir else None


# ---------------------------------------------------------------------------
# Step 3 - Grid map construction (offline)
# ---------------------------------------------------------------------------
def step3_grid_map(map_name: str, bag_dir: str):
    print("\n" + "=" * 60)
    print("Step 3: Grid map construction (offline)")
    print("=" * 60)
    print(f"  Map name : {map_name}")
    print(f"  Bag dir  : {bag_dir or '(not found, manual play needed)'}")

    # 1. Start relocalization
    print(f"\n[1/5] Starting relocalization with prior='{map_name}' ...")
    relocal_proc = launch(
        f"ros2 launch faster_lio slam.launch.py relocal:=true prior_dir:={map_name}"
    )
    _RUNNING_PROCESSES.append(relocal_proc)

    if not wait_for_node("/faster_lio", timeout=30):
        print("[WARN] /faster_lio node not detected after 30s.")
    else:
        print("[OK] /faster_lio node is running.")

    # 2. Start grid mapper
    print("\n[2/5] Starting global grid mapper + Rviz ...")
    grid_proc = launch(f"ros2 launch {GRIDMAPPER_LAUNCH}")
    _RUNNING_PROCESSES.append(grid_proc)
    pause("Press Enter after Rviz loads and you see the grid mapper ready.")

    # 3. Play bag
    bag_play_proc = None
    if bag_dir:
        print(f"\n[3/5] Playing bag '{bag_dir}' with --clock ...")
        bag_play_proc = launch(f"ros2 bag play {bag_dir}/ --clock")
        _RUNNING_PROCESSES.append(bag_play_proc)
    else:
        print("\n[3/5] Bag directory not found.")
        print("  Manually run: ros2 bag play <your_bag>/ --clock")
        pause("Press Enter after you start the bag play manually.")

    # 4. Observe
    print("\n[4/5] Observe the grid map in Rviz.")
    if bag_play_proc:
        pause("Press Enter when grid mapping is complete.")
        kill_process(bag_play_proc)
        _RUNNING_PROCESSES.remove(bag_play_proc)
    else:
        pause("Press Enter when grid mapping is complete.")

    # 5. Stop and collect
    print("\n[5/5] Stopping grid mapper ...")
    kill_process(grid_proc)
    _RUNNING_PROCESSES.remove(grid_proc)
    kill_process(relocal_proc)
    _RUNNING_PROCESSES.remove(relocal_proc)

    # Check output
    map_png = GRIDMAPPER_OUTPUT / "map.png"
    map_yaml = GRIDMAPPER_OUTPUT / "map.yaml"
    map_conn = GRIDMAPPER_OUTPUT / "map_connections.txt"

    print("\n[CHECK] Grid map output:")
    for f in (map_png, map_yaml, map_conn):
        status = "OK" if f.exists() else "MISSING"
        print(f"  [{status}] {f.name}")

    # Rename if needed
    if map_png.exists():
        if confirm(f"Rename map files to '{map_name}' ?", default=True):
            _rename_grid_map("map", map_name)
        else:
            print("[SKIP] Keeping default 'map' name.")

        # Review
        print(
            f"\n  Review {map_name}.png -- use GIMP to edit if needed"
            f" (do NOT change resolution)."
        )
        pause("Press Enter after you are satisfied with the map.")

        # Copy to maps directory
        if confirm(f"Copy map files to {MAPS_DIR}/ ?", default=True):
            _copy_grid_map(map_name)
        else:
            print("[SKIP] Map copy skipped.")
    else:
        print("\n[WARN] map.png not found. Skipping rename and copy.")

    # Rebuild?
    if confirm("Rebuild multi_map_nav_ros2 ?", default=False):
        print("\n[REBUILD] colcon building ...")
        run(
            f"cd {ALGOR_WS_ROOT} && colcon build --packages-select multi_map_nav_ros2"
        )
        print("[OK] Build complete.")

    pause("Step 3 complete.")


def _rename_grid_map(old_name: str, new_name: str):
    """Rename map files in the gridmapper output directory."""
    for ext in (".png", ".yaml", ".txt"):
        old_path = GRIDMAPPER_OUTPUT / f"{old_name}{ext}"
        new_path = GRIDMAPPER_OUTPUT / f"{new_name}{ext}"
        if old_path.exists():
            old_path.rename(new_path)
            print(f"  Renamed {old_path.name} -> {new_path.name}")

    # Update the image path inside the yaml
    yaml_path = GRIDMAPPER_OUTPUT / f"{new_name}.yaml"
    if yaml_path.exists():
        content = yaml_path.read_text()
        if f"image: {old_name}.png" in content:
            content = content.replace(
                f"image: {old_name}.png", f"image: {new_name}.png"
            )
            yaml_path.write_text(content)
            print(f"  Updated image path in {new_name}.yaml")


def _copy_grid_map(map_name: str):
    """Copy the final map files to the navigation maps directory."""
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    for ext in (".png", ".yaml"):
        src = GRIDMAPPER_OUTPUT / f"{map_name}{ext}"
        if src.exists():
            shutil.copy2(src, MAPS_DIR / src.name)
            print(f"  Copied {src.name} -> {MAPS_DIR}/")

    conn_src = GRIDMAPPER_OUTPUT / "map_connections.txt"
    if conn_src.exists():
        shutil.copy2(conn_src, MAPS_DIR / conn_src.name)
        print(f"  Copied map_connections.txt -> {MAPS_DIR}/")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    banner = r"""
============================================================
         Interactive SLAM Mapping Tool
============================================================
  Step 1  Sensor data acquisition (Lidar + IMU)
  Step 2  3D pointcloud map construction (PGO)
  Step 3  Grid map construction (offline)
============================================================"""
    print(banner)

    print(f"\n[INFO] Workspace: {ALGOR_WS_ROOT}")
    if not (ALGOR_WS_ROOT / "install" / "setup.bash").exists():
        print("[WARN] install/setup.bash not found. Source it or rebuild first.")

    if not confirm("Ready to start ?"):
        print("Aborted.")
        return

    # Step 1
    livox_proc, nav_proc = step1_sensor_setup()

    # Step 2
    map_name, bag_dir = step2_pointcloud_slam()

    # Step 3
    step3_grid_map(map_name, bag_dir)

    # Cleanup remaining sensor nodes
    print("\n" + "=" * 60)
    print("Mapping complete. Stopping remaining sensor nodes.")
    print("=" * 60)
    for p in (livox_proc, nav_proc):
        if p.poll() is None and p in _RUNNING_PROCESSES:
            kill_process(p)
            _RUNNING_PROCESSES.remove(p)

    print("\n[DONE] All done.")


if __name__ == "__main__":
    main()
