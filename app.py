#!/usr/bin/env python3
"""Streamlit UI for interactive ROS2 mapping workflow.

Layout:
  - Sidebar: workflow progress, paths, and termination control
  - Top row: mapping workflow and operational messages
  - Bottom: active screen session controls and terminal log
"""

import threading
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import json
import shlex
from datetime import datetime
from pathlib import Path

import streamlit as st

from i18n import TRANSLATIONS, t
from multimap import (
    TRANSITION_TYPES,
    archive_previous_output,
    deploy_project,
    inspect_multimap_dir,
    next_map_id_after,
    read_multimap_tables,
    switch_map_request,
)
from pcd_preview import load_xyz, preview_summary, topdown_image

# ---------------------------------------------------------------------------
# Page config
# ---------------------------------------------------------------------------
st.set_page_config(page_title="Multi-Floor Mapping", layout="wide", initial_sidebar_state="expanded")

# Language selector — must be before any t() call
if "lang" not in st.session_state:
    st.session_state.lang = "zh"

# ---------------------------------------------------------------------------
# Path constants
# ---------------------------------------------------------------------------
APP_DIR = Path(__file__).resolve().parent
HOME = Path.home()
ALGOR_WS = Path(os.environ.get("MAPPING_UI_WS_SRC", APP_DIR.parent)).expanduser()
ALGOR_WS_ROOT = ALGOR_WS.parent
FASTER_SLAM = ALGOR_WS / "faster-slam"
PGO_OUTPUT = FASTER_SLAM / "data" / "PGO_output"
PRIOR_DIR = FASTER_SLAM / "prior"
GRIDMAPPER_OUTPUT = ALGOR_WS / "gridmapper" / "data" / "Output"
MULTI_MAP_OUTPUT = GRIDMAPPER_OUTPUT / "multi_maps"
MAPS_ROOT = Path(os.environ.get("MAPPING_UI_MAPS_ROOT", ALGOR_WS_ROOT.parent / "Maps")).expanduser()
BAGS_DIR = HOME / "bags"
LOGS_DIR = ALGOR_WS_ROOT / "mapping_logs"

# ROS2 launch commands
LIVOX_LAUNCH_CMD = "ros2 launch livox_ros_driver2 msg_multi_MID360_launch.py model:=mid360s"
NAV_BRIDGE_LAUNCH_CMD = "ros2 launch nav_bridge nav_bridge.launch.py"
SLAM_PGO_LAUNCH_CMD = "ros2 launch faster_lio slam.launch.py pgo:=true rviz:=false"
RELOCAL_LAUNCH_CMD = "ros2 launch faster_lio slam.launch.py relocal:=true prior_dir:={prior}"
GRIDMAPPER_LAUNCH_CMD = "ros2 launch gridmapper global.launch.py rviz:=false"
CONTEXT_RECORDER_CMD = "ros2 run map_context_tracker map_context_recorder --ros-args -p output_dir:={output_dir}"

LIVOX_TOPIC = "/livox/lidar"
IMU_TOPIC = "/imu/data"
PGO_WAIT_TIMEOUT_SEC = 300
PGO_STABLE_POLLS = 5
PGO_EXIT_GRACE_SEC = 20
MAP_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,47}$")

# ---------------------------------------------------------------------------
# Environment
# ---------------------------------------------------------------------------


def _ros2_env() -> dict:
    """Source ROS2 setup files and capture the **full** resulting environment.

    Manually setting PATH/PYTHONPATH alone is insufficient — ROS2 also
    needs AMENT_PREFIX_PATH, LD_LIBRARY_PATH, COLCON_PREFIX_PATH, etc.
    The only reliable way is to source setup.bash and harvest ``env``.
    """
    setup_cmds: list[str] = []

    # Detect distro
    distro = os.environ.get("ROS_DISTRO", "")
    if not distro:
        opt_ros = Path("/opt/ros")
        if opt_ros.is_dir():
            candidates = sorted(d.name for d in opt_ros.iterdir() if d.is_dir())
            if candidates:
                distro = candidates[-1]  # pick latest if multiple
    if distro:
        ros_setup = Path(f"/opt/ros/{distro}/setup.bash")
        if ros_setup.exists():
            setup_cmds.append(f"source {ros_setup}")

    ws_setup = ALGOR_WS_ROOT / "install" / "setup.bash"
    if ws_setup.exists():
        setup_cmds.append(f"source {ws_setup}")

    if not setup_cmds:
        return os.environ.copy()

    # Source all setup files, then dump the resulting environment
    script = " && ".join(setup_cmds) + " && env -0"
    try:
        result = subprocess.run(
            ["bash", "-c", script],
            capture_output=True, text=True, timeout=10,
        )
        env: dict[str, str] = {}
        for entry in result.stdout.split("\0"):
            if "=" in entry:
                key, _, value = entry.partition("=")
                env[key] = value
        if env:
            return env
    except Exception:
        pass

    return os.environ.copy()


ROS2_ENV = _ros2_env()

# ---------------------------------------------------------------------------
# Screen session registry  (persisted in st.session_state)
# ---------------------------------------------------------------------------

KNOWN_SESSIONS = [
    {"name": "livox", "label": "Livox Lidar", "expected_nodes": ["/livox_lidar_publisher"]},
    {"name": "nav_bridge", "label": "nav_bridge IMU", "expected_nodes": ["/nav_bridge_node"]},
    {"name": "slam", "label": "PGO SLAM + Rviz", "expected_nodes": ["/laser_mapping"]},
    {"name": "bag_rec", "label": "Bag Recording", "expected_nodes": []},
    {"name": "relocal", "label": "Relocalization", "expected_nodes": ["/laser_mapping"]},
    {"name": "gridmapper", "label": "Grid Mapper + Rviz", "expected_nodes": ["/gridmapper_node"]},
    {"name": "map_context_recorder", "label": "Map Context Recorder", "expected_nodes": ["/map_context_recorder"]},
    {"name": "bag_play", "label": "Bag Playback", "expected_nodes": []},
    {"name": "build", "label": "colcon Build", "expected_nodes": []},
]

SESSION_LABEL = {s["name"]: s["label"] for s in KNOWN_SESSIONS}
SESSION_NAMES = [s["name"] for s in KNOWN_SESSIONS]
SCREEN_LIVE_STATES = ("Attached", "Detached", "Multi")


def _init_sessions():
    if "sessions" not in st.session_state:
        st.session_state.sessions = {}


def _register_session(name: str, cmd: str, log_file: str):
    st.session_state.sessions[name] = {"cmd": cmd, "log_file": log_file}


def _get_session(name: str) -> dict | None:
    return st.session_state.sessions.get(name)


def _screen_wipe_dead() -> None:
    """Remove dead screen sockets left behind after crashes or force kills."""
    try:
        subprocess.run(
            ["screen", "-wipe"],
            capture_output=True, text=True, timeout=5,
        )
    except Exception:
        pass


def _screen_entries(wipe_dead: bool = True) -> list[dict[str, str]]:
    """Parse `screen -ls`, optionally cleaning stale `Dead ???` sockets first."""
    if wipe_dead:
        _screen_wipe_dead()
    try:
        result = subprocess.run(
            ["screen", "-ls"],
            capture_output=True, text=True, timeout=5,
        )
    except Exception:
        return []

    entries: list[dict[str, str]] = []
    pattern = re.compile(r"^\s*(\d+)\.([^\s]+)\s+.*\(([^)]*)\)")
    for line in result.stdout.splitlines():
        m = pattern.match(line)
        if not m:
            continue
        state = m.group(3)
        entries.append({"pid": m.group(1), "name": m.group(2), "state": state})
    return entries


def _session_alive(name: str) -> bool:
    """Check whether a screen session is live, ignoring stale `Dead ???` sockets."""
    for entry in _screen_entries():
        if entry["name"] == name and any(state in entry["state"] for state in SCREEN_LIVE_STATES):
            return True
    return False


def _screen_quit(name: str) -> None:
    """Send quit to a screen session."""
    try:
        subprocess.run(f"screen -S {name} -X quit", shell=True, timeout=5)
    except Exception:
        pass


def _force_kill(name: str) -> None:
    try:
        for entry in _screen_entries():
            pid = entry["pid"]
            if entry["name"] == name and pid.isdigit():
                script = f"""
                kill_tree() {{
                    local parent=$1
                    local children=$(pgrep -P $parent)
                    for child in $children; do
                        kill_tree $child
                    done
                    kill -9 $parent 2>/dev/null
                }}
                kill_tree {pid}
                """
                subprocess.run(script, shell=True, executable='/bin/bash', timeout=5)
    except Exception:
        pass
    time.sleep(0.3)
    _screen_wipe_dead()


def _send_ctrl_c(name: str) -> None:
    try:
        subprocess.run(
            ["screen", "-S", name, "-p", "0", "-X", "stuff", "\x03"]
        )
    except Exception:
        pass


def screen_launch(name: str, cmd: str, cwd: str | None = None) -> str:
    _force_kill(name)
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = str(LOGS_DIR / f"{name}_{stamp}.log")
    stdbuf_cmd = f"stdbuf -oL -eL {cmd} > {log_file} 2>&1"
    if cwd:
        # cd first, then run the stdbuf-wrapped command
        full_cmd = f"cd {cwd} && {stdbuf_cmd}"
    else:
        full_cmd = stdbuf_cmd
    subprocess.run(
        f'screen -dmS {name} bash -c "{full_cmd}"',
        shell=True, env=ROS2_ENV,
    )
    time.sleep(0.5)
    _register_session(name, cmd, log_file)
    return log_file


def screen_stop(name: str, graceful: bool = True) -> None:
    """Stop a screen session.

    If graceful, send Ctrl+C first, then quit to survivors.
    Falls back to kill -9 if still alive.
    """
    if not _session_alive(name):
        return
    # Phase 1: Ctrl+C (SIGINT)
    if graceful:
        _send_ctrl_c(name)
        time.sleep(1)
    # Phase 2: quit
    if _session_alive(name):
        _screen_quit(name)
        time.sleep(0.5)
    # Phase 3: force kill
    if _session_alive(name):
        _force_kill(name)


def screen_restart(name: str) -> str | None:
    info = _get_session(name)
    if not info:
        return None
    _force_kill(name)
    return screen_launch(name, info["cmd"])


def clean_orphans() -> None:
    """Force kill all orphaned ROS2, RViz, Livox, and bag processes to free DDS domains."""
    patterns = [
        "faster_lio", "faster_pgo", "rviz2", "nav_bridge",
        "livox_ros_driver2", "gridmapper",
        "run_mapping_online_ros2"
    ]
    for p in patterns:
        try:
            subprocess.run(f"pkill -f -9 '{p}'", shell=True, timeout=2)
        except Exception:
            pass


def screen_stop_all() -> None:
    for name in list(st.session_state.sessions.keys()):
        _force_kill(name)
    clean_orphans()
    _screen_wipe_dead()


# ANSI escape sequence removal
_ANSI_RE = re.compile(r'\x1b\[[0-9;]*[a-zA-Z]|\x1b\]\d+[^a-z]*\x07')


def _strip_ansi(text: str) -> str:
    return _ANSI_RE.sub('', text)


def screen_read_log(name: str, max_lines: int = 100) -> str:
    info = _get_session(name)
    if not info or not info.get("log_file"):
        return "(session not registered)"
    log_path = Path(info["log_file"])
    if not log_path.exists():
        return "(log file not found)"
    try:
        # Fast native Python tail implementation using binary seek (0% CPU, 0 subprocesses)
        block_size = 65536
        with open(log_path, "rb") as f:
            try:
                f.seek(0, 2)
                file_size = f.tell()
                if file_size > block_size:
                    f.seek(-block_size, 2)
                else:
                    f.seek(0)
                data = f.read()
            except OSError:
                f.seek(0)
                data = f.read()
        text = data.decode("utf-8", errors="ignore")
        lines = text.splitlines()
        tail_lines = lines[-max_lines:]
        return _strip_ansi("\n".join(tail_lines)) or "(empty)"
    except Exception as e:
        return f"(error: {e})"


def screen_read_log_full(name: str) -> str:
    info = _get_session(name)
    if not info:
        return ""
    try:
        return _strip_ansi(Path(info["log_file"]).read_text())
    except FileNotFoundError:
        return ""


_init_sessions()

# ---------------------------------------------------------------------------
# ROS2 query helpers
# ---------------------------------------------------------------------------


def run_ros2_cmd(cmd: str, timeout: int = 10) -> str | None:
    import signal
    try:
        # Force PYTHONUNBUFFERED=1 to ensure that Python-based ROS2 CLI tools
        # flush stdout immediately when redirected to a pipe.
        env = ROS2_ENV.copy()
        env["PYTHONUNBUFFERED"] = "1"
        # start_new_session=True creates a new process group.
        # When timing out, we SIGKILL the entire group to guarantee that no orphaned ROS2 child nodes leak.
        proc = subprocess.Popen(
            cmd, shell=True, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, start_new_session=True
        )
        try:
            stdout, stderr = proc.communicate(timeout=timeout)
            return stdout.strip()
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
            stdout, stderr = proc.communicate()
            return stdout.strip()
    except Exception:
        return None


PACKAGE_DIR_HINTS = {
    "faster_lio": ["faster-slam", "faster_lio", "faster_slam"],
    "gridmapper": ["gridmapper"],
    "multi_map_nav": ["multi_map_nav_ros2", "multi_map_nav"],
}


def package_dir_candidates(pkg_name: str) -> list[str]:
    candidates = PACKAGE_DIR_HINTS.get(pkg_name, [])
    candidates += [pkg_name, pkg_name.replace("_", "-"), pkg_name.replace("-", "_")]
    return list(dict.fromkeys(candidates))


def workspace_src_from_prefix(prefix_path: Path) -> Path | None:
    """Infer a colcon workspace src directory from merged or isolated install prefixes."""
    resolved = prefix_path.resolve()
    if resolved.name == "install":
        workspace_root = resolved.parent
    elif resolved.parent.name == "install":
        workspace_root = resolved.parent.parent
    elif "install" in resolved.parts:
        install_idx = resolved.parts.index("install")
        workspace_root = Path(*resolved.parts[:install_idx])
    else:
        return None

    workspace_src = workspace_root / "src"
    return workspace_src if workspace_src.is_dir() else None


def find_package_in_src(workspace_src: Path, pkg_name: str, deep_scan: bool = False) -> Path | None:
    for name in package_dir_candidates(pkg_name):
        candidate_path = workspace_src / name
        if candidate_path.is_dir():
            return candidate_path

    if not deep_scan:
        return None

    try:
        for pattern in ["*/package.xml", "*/*/package.xml", "*/*/*/package.xml"]:
            for p in workspace_src.glob(pattern):
                try:
                    content = p.read_text(errors="ignore")
                    if re.search(r'<name>\s*' + re.escape(pkg_name) + r'\s*</name>', content):
                        return p.parent
                except Exception:
                    pass
    except Exception:
        pass

    return None


def iter_ros_prefixes() -> list[Path]:
    prefixes: list[Path] = []
    for env_key in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        for item in ROS2_ENV.get(env_key, "").split(os.pathsep):
            if item:
                path = Path(item)
                if path.exists():
                    prefixes.append(path)
    return list(dict.fromkeys(prefixes))


def workspace_src_candidates() -> list[Path]:
    candidates = [ALGOR_WS, APP_DIR.parent, Path.cwd().parent]
    return list(dict.fromkeys(p.resolve() for p in candidates if p.is_dir()))


def get_package_path(pkg_name: str) -> Path | None:
    for workspace_src in workspace_src_candidates():
        src_path = find_package_in_src(workspace_src, pkg_name)
        if src_path:
            return src_path

    for prefix_path in iter_ros_prefixes():
        resource = prefix_path / "share" / "ament_index" / "resource_index" / "packages" / pkg_name
        if not resource.exists():
            continue

        workspace_src = workspace_src_from_prefix(prefix_path)
        if workspace_src:
            src_path = find_package_in_src(workspace_src, pkg_name)
            if src_path:
                return src_path

        share_path = prefix_path / "share" / pkg_name
        if share_path.is_dir():
            return share_path

    for workspace_src in workspace_src_candidates():
        src_path = find_package_in_src(workspace_src, pkg_name, deep_scan=True)
        if src_path:
            return src_path

    return None


@st.cache_resource
def get_resolved_paths():
    start = time.monotonic()

    faster_lio_path = get_package_path("faster_lio")
    faster_lio = faster_lio_path if faster_lio_path else (ALGOR_WS / "faster-slam")

    gridmapper_path = get_package_path("gridmapper")
    gridmapper = gridmapper_path if gridmapper_path else (ALGOR_WS / "gridmapper")

    multi_map_nav_path = get_package_path("multi_map_nav")
    multi_map_nav = multi_map_nav_path if multi_map_nav_path else (ALGOR_WS / "multi_map_nav_ros2")

    res = {
        "FASTER_SLAM": faster_lio,
        "PGO_OUTPUT": faster_lio / "data" / "PGO_output",
        "PRIOR_DIR": faster_lio / "prior",
        "GRIDMAPPER_OUTPUT": gridmapper / "data" / "Output",
        "MULTI_MAP_OUTPUT": gridmapper / "data" / "Output" / "multi_maps",
    }

    print("--- ROS2 Package Path Resolution (Cached) ---")
    print(f"  FASTER_SLAM: {res['FASTER_SLAM']}")
    print(f"  PGO_OUTPUT: {res['PGO_OUTPUT']}")
    print(f"  GRIDMAPPER_OUTPUT: {res['GRIDMAPPER_OUTPUT']}")
    print(f"  MULTI_MAP_OUTPUT: {res['MULTI_MAP_OUTPUT']}")
    print(f"  resolved_in: {time.monotonic() - start:.3f}s")
    print("---------------------------------------------")
    return res


# Resolve directories dynamically at load-time (cached by Streamlit)
paths = get_resolved_paths()
FASTER_SLAM = paths["FASTER_SLAM"]
PGO_OUTPUT = paths["PGO_OUTPUT"]
PRIOR_DIR = paths["PRIOR_DIR"]
GRIDMAPPER_OUTPUT = paths["GRIDMAPPER_OUTPUT"]
MULTI_MAP_OUTPUT = paths["MULTI_MAP_OUTPUT"]


def check_topic_publishers(topic: str) -> int:
    output = run_ros2_cmd(f"ros2 topic info {topic}")
    if not output:
        return -1
    for line in output.splitlines():
        # Humble: "Publisher count: 1"  /  Jazzy: "Publisher count: 1"
        if "publisher" in line.lower() and "count" in line.lower():
            parts = line.split(":")
            if len(parts) >= 2:
                try:
                    return int(parts[-1].strip())
                except ValueError:
                    pass
    return 0


def check_node_exists(node_name: str) -> bool:
    output = run_ros2_cmd("ros2 node list", timeout=2)
    return output is not None and node_name in output


@st.cache_resource
def get_monitor_manager():
    class MonitorManager:
        def __init__(self):
            self.rates = {
                "/livox/lidar": 0.0,
                "/imu/data": 0.0,
                "/cloud_registered_body_horizon": 0.0,
                "/odometry_horizon": 0.0,
            }
            self.last_request = {}
            self.stop_event = threading.Event()
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()

        def _loop(self):
            while not self.stop_event.is_set():
                active_topics = []
                now = time.monotonic()
                for topic in list(self.rates.keys()):
                    # Only monitor if this topic was queried in the last 6 seconds
                    if now - self.last_request.get(topic, 0.0) < 6.0:
                        active_topics.append(topic)

                if not active_topics:
                    time.sleep(1.0)
                    continue

                for topic in active_topics:
                    if self.stop_event.is_set():
                        break

                    # 1. Skip querying if we already have a valid frequency (only need to detect once)
                    if self.rates[topic] > 0.0:
                        continue

                    # 2. Low-overhead pre-check: if no active publishers, rate is 0.0 instantly
                    if check_topic_publishers(topic) <= 0:
                        self.rates[topic] = 0.0
                        continue

                    # 3. Only run heavier ros2 topic hz when publisher exists.
                    # We wrap with timeout --signal=INT 4 so that it exits gracefully via Ctrl+C (SIGINT)
                    # after 2 seconds, which flushes its stdout buffer naturally.
                    output = run_ros2_cmd(
                        f"timeout --signal=INT 4 ros2 topic hz {topic}",
                        timeout=3,
                    )
                    rate = 0.0
                    if output:
                        for line in output.splitlines():
                            if "average rate:" in line:
                                try:
                                    rate = float(line.split("average rate:")[1].strip().split()[0])
                                    break
                                except (ValueError, IndexError):
                                    pass
                    self.rates[topic] = rate
                time.sleep(1.0)

        def get_hz(self, topic):
            self.last_request[topic] = time.monotonic()
            return self.rates.get(topic, 0.0)

        def reset(self):
            for topic in self.rates:
                self.rates[topic] = 0.0

    return MonitorManager()


def get_topic_hz(topic: str, timeout: int = 1) -> float:
    """Read topic rate from the resource-cached background monitor (completely non-blocking)."""
    manager = get_monitor_manager()
    return manager.get_hz(topic)


# ---------------------------------------------------------------------------
# File helpers
# ---------------------------------------------------------------------------


def find_bag_dir(bag_name: str) -> str | None:
    exact = BAGS_DIR / bag_name
    if exact.is_dir():
        return str(exact)
    for p in sorted(BAGS_DIR.glob(f"{bag_name}_*")):
        if p.is_dir():
            return str(p)
    return None


def get_bag_duration_sec(bag_dir: str | None) -> float | None:
    if not bag_dir:
        return None
    metadata = Path(bag_dir) / "metadata.yaml"
    if not metadata.exists():
        return None
    try:
        with metadata.open("r", encoding="utf-8", errors="ignore") as f:
            lines = f.read(65536).splitlines()
    except OSError:
        return None

    in_duration = False
    duration_indent = 0
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        indent = len(line) - len(line.lstrip())
        if stripped.startswith("duration:"):
            value = stripped.split(":", 1)[1].strip()
            if value.isdigit():
                return int(value) / 1_000_000_000
            in_duration = True
            duration_indent = indent
            continue

        if in_duration:
            if indent <= duration_indent and not stripped.startswith("nanoseconds:"):
                in_duration = False
                continue
            if stripped.startswith("nanoseconds:"):
                value = stripped.split(":", 1)[1].strip()
                if value.isdigit():
                    return int(value) / 1_000_000_000

    return None


def copy_pgo_to_prior(map_name: str) -> list[str]:
    messages = []
    pgo_pcd = PGO_OUTPUT / "PGO.pcd"
    pgo_kf = PGO_OUTPUT / "keyframes"
    if not pgo_pcd.exists():
        return [t("msg_error_not_found", path=pgo_pcd)]
    if not pgo_kf.is_dir():
        return [t("msg_error_not_found", path=pgo_kf)]
    dest = PRIOR_DIR / map_name
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(pgo_pcd, dest / "PGO.pcd")
    messages.append(t("msg_copied_to", name="PGO.pcd", dest=dest))
    if (dest / "keyframes").exists():
        shutil.rmtree(dest / "keyframes")
    shutil.copytree(pgo_kf, dest / "keyframes")
    messages.append(t("msg_copied_to", name="keyframes/", dest=dest / "keyframes"))
    return messages


def archive_existing_pgo_output() -> list[str]:
    messages = []
    existing = [p for p in (PGO_OUTPUT / "PGO.pcd", PGO_OUTPUT / "keyframes") if p.exists()]
    if not existing:
        return messages

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    archive_dir = PGO_OUTPUT / f"archive_{stamp}"
    archive_dir.mkdir(parents=True, exist_ok=True)
    for path in existing:
        dest = archive_dir / path.name
        path.rename(dest)
        messages.append(t("msg_archived_old", name=path.name, dest=archive_dir))
    return messages


# ---------------------------------------------------------------------------
# Workflow state
# ---------------------------------------------------------------------------

def _step_defs():
    return [
        {"id": 0, "name": t("step0_name")},
        {"id": 1, "name": t("step1_name")},
        {"id": 2, "name": t("step2_name")},
        {"id": 3, "name": t("step3_name")},
        {"id": 4, "name": t("step4_name")},
    ]


def _init_state():
    defaults = {
        "current_step": 0,
        "current_sub": "start",
        "map_name": "",
        "bag_name": "",
        "bag_dir": None,
        "playback_started_at": None,
        "playback_duration_sec": None,
        "step_messages": [],
        "step1_livox_hz": 0.0,
        "step1_imu_hz": 0.0,
        "pgo_last_size": -1,
        "pgo_stable_count": 0,
        "pgo_files_stable_at": None,
        "active_map_id": "map_000",
        "switch_history": [],
        "last_switch_response": "",
        "selected_session": KNOWN_SESSIONS[0]["name"],
        "action_in_progress": False,
        "last_sub": "start",
        "last_step": 0,
    }
    for k, v in defaults.items():
        if k not in st.session_state:
            st.session_state[k] = v


_init_state()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def add_message(msg: str):
    stamp = datetime.now().strftime("%H:%M:%S")
    st.session_state.step_messages.insert(0, f"[{stamp}] {msg}")
    st.session_state.step_messages = st.session_state.step_messages[:50]


def record_switch_result(target_map: str, response: str | None) -> None:
    """Keep service diagnostics visible after Streamlit reruns."""
    result = response or "(no response from ros2 service call)"
    entry = {"time": datetime.now().strftime("%H:%M:%S"), "target": target_map, "response": result}
    st.session_state.switch_history.insert(0, entry)
    st.session_state.switch_history = st.session_state.switch_history[:20]
    st.session_state.last_switch_response = result


def get_wait_start():
    wait_start = st.session_state.get("wait_start")
    if wait_start is None:
        wait_start = time.monotonic()
        st.session_state.wait_start = wait_start
    return wait_start


def clear_wait_state():
    if "wait_start" in st.session_state:
        del st.session_state.wait_start


def render_initial_preparation(next_phase: str) -> bool:
    """Render the one-time sensor and platform-control preparation sequence.

    Returns True while the sequence owns the current workflow phase.
    """
    sub = st.session_state.current_sub
    phases = {
        "start_livox": "first_livox",
        "wait_livox": "wait_first_livox",
        "start_nav": "first_nav",
        "wait_nav": "wait_first_nav",
        "release": "first_release",
    }
    if sub == phases["start_livox"]:
        st.markdown(t("two_loop_first_sensors_desc"))
        if st.button(t("s1_start_livox"), type="primary", key="first_livox"):
            add_message(t("msg_start_livox"))
            screen_launch("livox", LIVOX_LAUNCH_CMD)
            st.session_state.current_sub = phases["wait_livox"]
            st.session_state.wait_start = time.monotonic()
            st.rerun()
        return True
    if sub == phases["wait_livox"]:
        elapsed = max(0.0, time.monotonic() - get_wait_start())
        st.progress(min(elapsed / 20, 1.0))
        st.caption(t("s1_wait_topic", topic=LIVOX_TOPIC, elapsed=elapsed))
        if get_topic_hz(LIVOX_TOPIC) > 0 or elapsed >= 20:
            st.session_state.current_sub = phases["start_nav"]
            clear_wait_state()
            st.rerun()
        return True
    if sub == phases["start_nav"]:
        st.markdown(t("s1_start_nav_desc"))
        if st.button(t("s1_start_nav"), type="primary", key="first_nav"):
            add_message(t("msg_start_nav"))
            screen_launch("nav_bridge", NAV_BRIDGE_LAUNCH_CMD)
            st.session_state.current_sub = phases["wait_nav"]
            st.session_state.wait_start = time.monotonic()
            st.rerun()
        return True
    if sub == phases["wait_nav"]:
        elapsed = max(0.0, time.monotonic() - get_wait_start())
        st.progress(min(elapsed / 20, 1.0))
        st.caption(t("s1_wait_topic", topic=IMU_TOPIC, elapsed=elapsed))
        if get_topic_hz(IMU_TOPIC) > 0 or elapsed >= 20:
            st.session_state.current_sub = phases["release"]
            clear_wait_state()
            st.rerun()
        return True
    if sub == phases["release"]:
        st.markdown(t("s1_release_desc"))
        if st.button(t("s1_release"), type="primary", key="first_release"):
            output = run_ros2_cmd("ros2 service call /nav_bridge_node/release_control std_srvs/srv/Trigger")
            add_message(t("msg_release_done", output=output or t("ok")))
            st.session_state.current_sub = next_phase
            st.rerun()
        return True
    return False


def render_node_wait(node_name: str, elapsed: float, timeout: int = 30) -> None:
    """Render the consistent bounded wait state used for ROS node startup."""
    st.progress(min(elapsed / timeout, 1.0))
    st.caption(t("s3_wait_node", node=node_name,
               elapsed=elapsed, seconds=timeout))


def render_runtime_status(**sessions: str) -> None:
    """Present active mapping processes in a compact, consistent status row."""
    columns = st.columns(len(sessions))
    for column, (label_key, session_name) in zip(columns, sessions.items()):
        column.caption(f"{t(label_key)}: {status_text(_session_alive(session_name))}")


def is_valid_map_name(name: str) -> bool:
    return bool(MAP_NAME_RE.fullmatch(name))


def format_duration(seconds: float | int | None) -> str:
    if seconds is None:
        return "unknown"
    seconds = max(0, int(round(seconds)))
    minutes, sec = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    if hours:
        return f"{hours:d}:{minutes:02d}:{sec:02d}"
    return f"{minutes:d}:{sec:02d}"


def status_text(alive: bool) -> str:
    return t("status_running") if alive else t("status_stopped")


def file_size_human(p: Path) -> str:
    size = p.stat().st_size
    if size < 1024:
        return f"{size} B"
    if size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    return f"{size / 1024 / 1024:.1f} MB"


def path_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    total = 0
    for child in path.rglob("*"):
        try:
            if child.is_file():
                total += child.stat().st_size
        except OSError:
            pass
    return total


def step_status(step_id: int) -> str:
    if st.session_state.current_step > step_id:
        return "done"
    if st.session_state.current_step == step_id:
        return "running"
    return "not_started"


def sync_action_lock() -> None:
    """Release button lock after a workflow step/sub-step transition."""
    if "last_sub" not in st.session_state or st.session_state.last_sub != st.session_state.current_sub:
        st.session_state.last_sub = st.session_state.current_sub
        st.session_state.action_in_progress = False

    if "last_step" not in st.session_state or st.session_state.last_step != st.session_state.current_step:
        st.session_state.last_step = st.session_state.current_step
        st.session_state.action_in_progress = False


# ---------------------------------------------------------------------------
# Sidebar
# ---------------------------------------------------------------------------


def render_sidebar():
    st.sidebar.title(t("sidebar_title"))

    # Language toggle
    lang_options = {"en": "English", "zh": "中文"}
    current_lang = st.session_state.lang
    selected_lang = st.sidebar.radio(
        f"🌐 {t('language')}",
        options=list(lang_options.keys()),
        format_func=lambda k: lang_options[k],
        index=list(lang_options.keys()).index(current_lang),
        horizontal=True,
        key="lang_radio",
    )
    if selected_lang != current_lang:
        st.session_state.lang = selected_lang
        st.rerun()

    st.sidebar.divider()
    st.sidebar.progress(st.session_state.current_step / 4)
    st.sidebar.caption(t("workflow_progress", current=st.session_state.current_step, total=4))

    for sdef in _step_defs():
        sid = sdef["id"]
        status = step_status(sid)
        if status == "done":
            st.sidebar.markdown(f">&#10004; **{sid}. {sdef['name']}**")
        elif status == "running":
            st.sidebar.markdown(f">&#9654; **{sid}. {sdef['name']}**")
        else:
            st.sidebar.markdown(f"&nbsp;&nbsp;{sid}. {sdef['name']}")

    st.sidebar.divider()

    with st.sidebar.expander(t("file_paths"), expanded=False):
        mn = st.session_state.map_name or t("tbd")
        bd = st.session_state.bag_dir or t("tbd")
        st.markdown(
            f"**{t('path_pgo_output')}:** `{PGO_OUTPUT}`\n\n"
            f"**{t('path_prior')}:** `{PRIOR_DIR}/{mn}`\n\n"
            f"**{t('path_bag')}:** `{bd}`\n\n"
            f"**{t('path_grid_map')}:** `{MULTI_MAP_OUTPUT}`\n\n"
            f"**{t('path_nav_maps')}:** `{MAPS_ROOT}/{mn}`"
        )

    if 0 < st.session_state.current_step < 4:
        st.sidebar.divider()
        if st.sidebar.button(t("abort_mapping"), type="primary", key="sidebar_abort"):
            screen_stop_all()
            get_monitor_manager().reset()
            add_message(t("abort_done"))
            st.session_state.current_step = 0
            st.session_state.current_sub = "start"
            st.rerun()


# ---------------------------------------------------------------------------
# LEFT PANEL - Session management (independent of workflow)
# ---------------------------------------------------------------------------


def _list_screen_sessions() -> list[str]:
    """Return names of currently active screen sessions via `screen -ls`."""
    names: list[str] = []
    for entry in _screen_entries():
        if any(state in entry["state"] for state in SCREEN_LIVE_STATES):
            names.append(entry["name"])
    return names


def render_session_bar():
    """Horizontal session management bar."""
    live_sessions = _list_screen_sessions()

    if not live_sessions:
        st.caption(t("no_active"))
        return None

    cols = st.columns([3, 1, 1, 1, 1])

    with cols[0]:
        default_idx = 0
        if st.session_state.selected_session in live_sessions:
            default_idx = live_sessions.index(st.session_state.selected_session)
        selected = st.selectbox(
            t("select_session"),
            options=live_sessions,
            index=default_idx,
            format_func=lambda n: SESSION_LABEL.get(n, n),
            label_visibility="collapsed",
        )
        st.session_state.selected_session = selected

    info = _get_session(selected)

    with cols[1]:
        if st.button(t("stop"), key=f"bar_stop_{selected}", type="primary", width="stretch"):
            screen_stop(selected)
            add_message(t("msg_stopped", name=selected))
            st.rerun()
    with cols[2]:
        if st.button(t("restart"), key=f"bar_restart_{selected}", width="stretch"):
            result = screen_restart(selected)
            if result:
                add_message(t("msg_restarted", name=selected))
            else:
                add_message(t("msg_cannot_restart", name=selected))
            st.rerun()
    with cols[3]:
        if st.button(t("refresh"), key=f"bar_refresh_{selected}", width="stretch"):
            st.rerun()
    with cols[4]:
        if info and info["log_file"] and Path(info["log_file"]).exists():
            st.download_button(
                t("download_log"),
                data=screen_read_log_full(selected),
                file_name=f"{selected}.log",
                mime="text/plain",
                key=f"bar_dl_{selected}",
                width="stretch",
            )

    # Show the command this session is running
    if info:
        st.caption(t("command", cmd=info["cmd"]))

    return selected


@st.fragment(run_every=1)
def _render_log_viewer(session_name: str):
    """Log viewer as a Streamlit fragment — only this section re-renders."""
    log_text = screen_read_log(session_name, max_lines=200)
    st.code(log_text, language="text", height=400)

# ---------------------------------------------------------------------------
# RIGHT PANEL - Mapping workflow
# ---------------------------------------------------------------------------


def render_messages():
    """Fixed-height message panel with auto-scroll to latest."""
    messages = st.session_state.step_messages
    st.markdown(f"**{t('messages')}**")
    with st.container(height=350):
        if not messages:
            st.caption("—")
        else:
            for msg in messages:
                if "ERROR" in msg or "错误" in msg:
                    st.error(msg)
                elif "WARN" in msg or "警告" in msg:
                    st.warning(msg)
                elif any(kw in msg for kw in ("OK", "Copied", "Renamed", "ready", "已复制", "已重命名", "已就绪", "正常")):
                    st.success(msg)
                else:
                    st.text(msg)


def render_step0():
    st.header(t("s0_header"))
    get_monitor_manager().reset()

    setup_path = ALGOR_WS_ROOT / "install" / "setup.bash"
    ws_info = t("workspace_info", workspace=ALGOR_WS_ROOT, distro=ROS2_ENV.get("ROS_DISTRO", "N/A"))
    if not setup_path.exists():
        st.warning(t("s0_setup_missing"))
    st.info(ws_info)

    st.markdown(t("s0_desc"))

    if st.button(t("start_workflow"), type="primary", key="btn_step0_start", disabled=st.session_state.action_in_progress):
        st.session_state.action_in_progress = True
        st.session_state.current_step = 1
        st.session_state.current_sub = "first_livox"
        st.session_state.step_messages = []
        st.rerun()


def render_multimap_controls() -> None:
    """Show live GridMapper output and safely call its map-switch service."""
    report = inspect_multimap_dir(
        MULTI_MAP_OUTPUT,
        allowed_unexported_map_ids={st.session_state.active_map_id},
    )
    st.subheader(t("s3_multimap_header"))
    st.caption(t("s3_multimap_caption", output=MULTI_MAP_OUTPUT))
    st.write(t("s3_multimap_summary", maps=", ".join(report.map_ids) or "—",
               relations=report.relations_count, transitions=report.transitions_count))
    relations, transitions = read_multimap_tables(MULTI_MAP_OUTPUT)
    if relations:
        with st.expander(t("s3_relations"), expanded=True):
            st.dataframe(relations, width="stretch", hide_index=True)
    if transitions:
        with st.expander(t("s3_transitions"), expanded=True):
            st.dataframe(transitions, width="stretch", hide_index=True)
    if st.session_state.switch_history:
        with st.expander(t("switch_history"), expanded=True):
            st.dataframe(st.session_state.switch_history, width="stretch", hide_index=True)
            st.caption(t("last_switch_response"))
            st.code(st.session_state.last_switch_response, language="text")
    if report.errors and MULTI_MAP_OUTPUT.exists():
        st.warning("; ".join(report.errors))

    pending_target = st.session_state.pop("pending_switch_target", None)
    if pending_target:
        st.session_state.switch_target_map = pending_target
    suggested = next_map_id_after(st.session_state.active_map_id)
    if not st.session_state.get("switch_target_map", "").strip():
        st.session_state.switch_target_map = suggested
    c1, c2, c3 = st.columns([2, 2, 1])
    with c1:
        target_map = st.text_input(t("s3_target_map"), key="switch_target_map")
    with c2:
        transition_type = st.selectbox(t("s3_transition_type"), TRANSITION_TYPES, key="switch_transition_type")
    with c3:
        bidirectional = st.checkbox(t("s3_bidirectional"), value=True, key="switch_bidirectional")
    if st.button(t("s3_switch_map"), type="primary", key="btn_s3_switch"):
        try:
            target_map = target_map.strip()
            if target_map == st.session_state.active_map_id:
                raise ValueError(t("switch_same_map", map=target_map))
            request = switch_map_request(target_map, transition_type, bidirectional)
            command = "ros2 service call /switch_map gridmapper/srv/SwitchMap " + shlex.quote(json.dumps(request))
            output = run_ros2_cmd(command, timeout=15)
            record_switch_result(target_map, output)
            if output and re.search(r"success\s*[:=]\s*true", output, re.IGNORECASE):
                st.session_state.active_map_id = target_map
                st.session_state.pending_switch_target = next_map_id_after(target_map)
                add_message(t("msg_switch_ok", target=target_map))
            elif output and "No synchronized odometry has been received yet" in output:
                add_message(t("msg_switch_not_ready"))
            else:
                add_message(t("msg_switch_failed", output=output or t("not_found")))
        except ValueError as exc:
            st.error(str(exc))
        st.rerun()
    st.caption(t("s3_active_map", map=st.session_state.active_map_id))


def render_step4():
    st.header(t("s4_header"))
    sub = st.session_state.current_sub

    if sub == "cleanup":
        sessions = list(st.session_state.sessions.items())
        running = [(n, i) for n, i in sessions if _session_alive(n)]

        if running:
            names = ", ".join(n for n, _ in running)
            st.markdown(t("s4_remaining", names=names))
            if st.button(t("s4_stop_remaining"), type="primary", key="btn_s4_stop", disabled=st.session_state.action_in_progress):
                st.session_state.action_in_progress = True
                for n, _ in running:
                    screen_stop(n)
                    add_message(t("msg_stopped", name=n))
                clean_orphans()
                st.session_state.current_sub = "done"
                st.rerun()
        else:
            st.session_state.current_sub = "done"
            st.rerun()

    if sub == "done":
        st.markdown(t("s4_all_done"))
        destination = MAPS_ROOT / st.session_state.map_name
        st.success(t("s4_project_location", path=destination))
        st.caption(t("s4_project_contents"))

        if st.button(t("s4_reset"), type="primary", key="btn_s4_reset", disabled=st.session_state.action_in_progress):
            st.session_state.action_in_progress = True
            screen_stop_all()
            get_monitor_manager().reset()
            if LOGS_DIR.exists():
                shutil.rmtree(LOGS_DIR, ignore_errors=True)
                LOGS_DIR.mkdir(parents=True, exist_ok=True)
            # Reset only workflow state, keep sessions registry
            workflow_keys = [
                "current_step", "current_sub", "map_name", "bag_name",
                "bag_dir", "playback_started_at", "playback_duration_sec",
                "step_messages", "step1_livox_hz", "step1_imu_hz",
                "pgo_last_size", "pgo_stable_count", "pgo_files_stable_at",
                "active_map_id",
                "switch_target_map",
                "switch_history", "last_switch_response",
            ]
            for k in workflow_keys:
                if k in st.session_state:
                    del st.session_state[k]
            _init_state()
            st.rerun()


# ---------------------------------------------------------------------------
# Two-loop mapping workflow
# ---------------------------------------------------------------------------


def render_first_loop_sensors():
    """First-loop sensor preparation."""
    st.header(t("two_loop_first_sensors"))
    if render_initial_preparation("first_name"):
        return
    st.session_state.current_step = 2
    if not st.session_state.map_name:
        st.session_state.map_name = datetime.now().strftime("sensor_%y%m%d_%H%M%S")
    st.rerun()


def _render_pcd_review(pcd_path: Path) -> None:
    st.subheader(t("two_loop_pcd_review"))
    if not pcd_path.is_file():
        st.error(t("msg_error_not_found", path=pcd_path))
        return
    try:
        xyz = load_xyz(pcd_path)
        summary = preview_summary(xyz)
        columns = st.columns(4)
        columns[0].caption(f"{t('two_loop_pcd_points')}：{summary['points']:,}")
        columns[1].caption(f"X：{summary['x_min']:.1f} ~ {summary['x_max']:.1f} m")
        columns[2].caption(f"Y：{summary['y_min']:.1f} ~ {summary['y_max']:.1f} m")
        columns[3].caption(f"Z：{summary['z_min']:.1f} ~ {summary['z_max']:.1f} m")
        st.image(topdown_image(xyz), caption=t("two_loop_pcd_topdown"), width="stretch")
    except (OSError, ValueError) as exc:
        st.warning(t("two_loop_pcd_external", path=pcd_path, error=exc))


def render_first_loop_pgo():
    """First loop: PGO + diagnostic bag, then explicit PCD confirmation."""
    st.header(t("two_loop_first_pgo"))
    sub, project = st.session_state.current_sub, st.session_state.map_name
    if sub == "first_name":
        name = st.text_input(t("s2_map_name"), value=project, key="two_project_name").strip()
        st.warning(t("two_loop_project_notice"))
        if st.button(t("s2_confirm_name"), type="primary", key="two_name_confirm"):
            if not is_valid_map_name(name):
                st.error(t("s2_name_invalid"))
            else:
                st.session_state.map_name, st.session_state.current_sub = name, "first_stand"
                add_message(t("s2_name_confirmed", name=name))
                st.rerun()
    elif sub == "first_stand":
        st.markdown(t("s2_stand_desc"))
        if st.button(t("s2_stand_btn"), type="primary", key="two_first_stand"):
            st.session_state.current_sub = "first_slam"
            st.rerun()
    elif sub == "first_slam":
        st.markdown(t("two_loop_first_slam_desc"))
        if st.button(t("two_loop_start_first_slam"), type="primary", key="two_first_slam"):
            for message in archive_existing_pgo_output():
                add_message(message)
            add_message(t("msg_start_slam"))
            screen_launch("slam", SLAM_PGO_LAUNCH_CMD)
            st.session_state.current_sub, st.session_state.wait_start = "wait_first_slam", time.monotonic()
            st.rerun()
    elif sub == "wait_first_slam":
        elapsed = max(0.0, time.monotonic() - get_wait_start())
        render_node_wait("laser_mapping", elapsed)
        if check_node_exists("laser_mapping") or elapsed >= 30:
            bag_name = f"{project}_first_sensor"
            BAGS_DIR.mkdir(parents=True, exist_ok=True)
            screen_launch("bag_rec", f"ros2 bag record -o {shlex.quote(bag_name)} {LIVOX_TOPIC} {IMU_TOPIC}", cwd=str(BAGS_DIR))
            st.session_state.bag_name = bag_name
            add_message(t("msg_record_to", path=f"{BAGS_DIR}/{bag_name}"))
            st.session_state.current_sub = "first_drive"
            clear_wait_state()
            st.rerun()
    elif sub == "first_drive":
        st.info(t("two_loop_first_drive_desc"))
        render_runtime_status(s2_slam_status="slam", s2_bag_recording_status="bag_rec")
        if st.button(t("two_loop_finish_first"), type="primary", key="two_first_finish"):
            screen_stop("bag_rec")
            add_message(t("msg_bag_stopped"))
            if _session_alive("slam"):
                _send_ctrl_c("slam")
                add_message(t("msg_slam_sigint"))
            st.session_state.pgo_last_size = -1
            st.session_state.pgo_stable_count = 0
            st.session_state.current_sub, st.session_state.wait_start = "wait_first_pgo", time.monotonic()
            st.rerun()
    elif sub == "wait_first_pgo":
        elapsed = max(0.0, time.monotonic() - get_wait_start())
        st.progress(min(elapsed / PGO_WAIT_TIMEOUT_SEC, 1.0))
        st.caption(t("s2_wait_pgo", elapsed=elapsed, seconds=PGO_WAIT_TIMEOUT_SEC))
        pcd, keyframes = PGO_OUTPUT / "PGO.pcd", PGO_OUTPUT / "keyframes"
        if pcd.is_file() and keyframes.is_dir():
            size = pcd.stat().st_size + path_size_bytes(keyframes)
            st.session_state.pgo_stable_count = st.session_state.pgo_stable_count + 1 if size == st.session_state.pgo_last_size else 0
            st.session_state.pgo_last_size = size
            st.caption(t("s2_size_stability", count=st.session_state.pgo_stable_count, target=PGO_STABLE_POLLS))
            if st.session_state.pgo_stable_count >= PGO_STABLE_POLLS and not _session_alive("slam"):
                st.session_state.current_sub = "pcd_review"
                clear_wait_state()
                add_message(t("msg_pgo_ready"))
                st.rerun()
        elif elapsed >= PGO_WAIT_TIMEOUT_SEC:
            st.error(t("msg_pgo_timeout", seconds=PGO_WAIT_TIMEOUT_SEC))
    elif sub == "pcd_review":
        _render_pcd_review(PGO_OUTPUT / "PGO.pcd")
        st.info(t("two_loop_pcd_confirm_desc"))
        if st.button(t("two_loop_confirm_pcd"), type="primary", key="two_confirm_pcd"):
            messages = copy_pgo_to_prior(project)
            for message in messages:
                add_message(message)
            if any(message.startswith(("ERROR:", "错误：")) for message in messages):
                return
            for name in ("bag_rec", "slam"):
                screen_stop(name)
            st.session_state.current_step, st.session_state.current_sub = 3, "second_relocal"
            st.rerun()


def render_second_loop_mapping():
    """Second loop: relocalize against confirmed PGO, then map with GridMapper."""
    st.header(t("two_loop_second_header"))
    sub, project = st.session_state.current_sub, st.session_state.map_name
    prior = PRIOR_DIR / project
    if not (prior / "PGO.pcd").is_file():
        st.error(t("msg_error_not_found", path=prior / "PGO.pcd"))
        return
    if sub == "second_relocal":
        st.markdown(t("two_loop_start_relocal_desc", prior=prior))
        st.warning(t("s3_archive_notice", output=MULTI_MAP_OUTPUT))
        if st.button(t("two_loop_start_relocal"), type="primary", key="two_second_relocal"):
            screen_launch("relocal", RELOCAL_LAUNCH_CMD.format(prior=prior))
            st.session_state.current_sub, st.session_state.wait_start = "wait_second_relocal", time.monotonic()
            st.rerun()
    elif sub == "wait_second_relocal":
        elapsed = max(0.0, time.monotonic() - get_wait_start())
        render_node_wait("laser_mapping", elapsed)
        if check_node_exists("laser_mapping") or elapsed >= 30:
            archived = archive_previous_output(GRIDMAPPER_OUTPUT)
            if archived:
                add_message(t("msg_multimap_archived", path=archived))
            st.session_state.active_map_id = "map_000"
            st.session_state.switch_target_map = next_map_id_after("map_000")
            screen_launch("gridmapper", GRIDMAPPER_LAUNCH_CMD)
            try:
                screen_launch("map_context_recorder", CONTEXT_RECORDER_CMD.format(output_dir=shlex.quote(str(MULTI_MAP_OUTPUT))))
                add_message(t("msg_context_recorder_started"))
            except OSError as exc:
                add_message(t("msg_context_recorder_failed", error=exc))
            add_message(t("msg_start_grid"))
            st.session_state.current_sub, st.session_state.wait_start = "wait_second_grid", time.monotonic()
            st.rerun()
    elif sub == "wait_second_grid":
        elapsed = max(0.0, time.monotonic() - get_wait_start())
        render_node_wait("gridmapper_node", elapsed)
        if check_node_exists("gridmapper_node") or elapsed >= 30:
            st.session_state.current_sub = "second_wait_inputs"
            clear_wait_state()
            st.rerun()
    elif sub == "second_wait_inputs":
        cloud_hz = get_topic_hz("/cloud_registered_body_horizon")
        odom_hz = get_topic_hz("/odometry_horizon")
        st.markdown(t("two_loop_input_status", cloud_hz=cloud_hz, odom_hz=odom_hz))
        if cloud_hz > 0 and odom_hz > 0:
            st.success(t("two_loop_input_ready"))
            st.session_state.current_sub = "second_drive"
            st.rerun()
        else:
            st.warning(t("two_loop_input_wait"))
            st.caption(t("two_loop_input_diagnose"))
    elif sub == "second_drive":
        st.info(t("two_loop_second_drive_desc"))
        render_runtime_status(s3_relocal_status="relocal", s3_grid_status="gridmapper")
        render_multimap_controls()
        if st.button(t("two_loop_finish_second"), type="primary", key="two_second_finish"):
            if _session_alive("gridmapper"):
                _send_ctrl_c("gridmapper")
                add_message(t("msg_send_grid_sigint"))
            if _session_alive("map_context_recorder"):
                screen_stop("map_context_recorder")
                add_message(t("msg_context_recorder_stopped"))
            st.session_state.current_sub, st.session_state.wait_start = "wait_second_output", time.monotonic()
            st.rerun()
    elif sub == "wait_second_output":
        elapsed = max(0.0, time.monotonic() - get_wait_start())
        st.progress(min(elapsed / 45, 1.0))
        report = inspect_multimap_dir(MULTI_MAP_OUTPUT)
        if not _session_alive("gridmapper") or elapsed >= 45:
            if _session_alive("gridmapper"):
                screen_stop("gridmapper")
            if not report.valid:
                st.session_state.current_sub = "second_validation_error"
                st.rerun()
            st.session_state.current_sub = "second_review"
            clear_wait_state()
            add_message(t("msg_grid_ready"))
            st.rerun()
    elif sub == "second_review":
        report = inspect_multimap_dir(MULTI_MAP_OUTPUT)
        if not report.valid:
            st.session_state.current_sub = "second_validation_error"
            st.rerun()
        st.success(t("two_loop_maps_valid", maps=", ".join(report.map_ids), relations=report.relations_count, transitions=report.transitions_count))
        context_path = MULTI_MAP_OUTPUT / "context" / "inspection_context.yaml"
        if context_path.is_file():
            st.success(t("two_loop_context_ready", path=context_path))
        else:
            st.warning(t("two_loop_context_missing", path=context_path))
        selected_map = st.selectbox(t("two_loop_preview_floor"), report.map_ids, key="second_preview_map")
        st.image(str(MULTI_MAP_OUTPUT / f"{selected_map}.png"), caption=t("two_loop_preview_caption", map_id=selected_map), width="stretch")
        st.info(t("two_loop_review_maps_desc", destination=MAPS_ROOT / project))
        if st.button(t("two_loop_confirm_deploy"), type="primary", key="two_confirm_deploy"):
            try:
                target = deploy_project(prior, MULTI_MAP_OUTPUT, MAPS_ROOT, project)
                add_message(t("msg_project_deployed", path=target))
                st.session_state.current_step, st.session_state.current_sub = 4, "cleanup"
                st.rerun()
            except (OSError, ValueError) as exc:
                st.error(str(exc))
    elif sub == "second_validation_error":
        for error in inspect_multimap_dir(MULTI_MAP_OUTPUT).errors:
            st.error(error)
        if st.button(t("two_loop_recheck"), key="two_recheck"):
            st.session_state.current_sub = "wait_second_output"
            st.session_state.wait_start = time.monotonic()
            st.rerun()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    sync_action_lock()

    st.title(t("page_title"))
    render_sidebar()

    # --- Browser-level refresh guard while workflow is active ---
    if 0 < st.session_state.current_step < 4:
        st.html("""
        <script>
        window.addEventListener('beforeunload', function(e) {
            e.preventDefault();
            e.returnValue = '';
        });
        </script>
        """)

    # --- In-page warning: detect stale sessions on fresh page load ---
    if st.session_state.current_step == 0:
        live = _list_screen_sessions()
        known_live = [n for n in live if n in SESSION_NAMES]
        if known_live and "refresh_dismissed" not in st.session_state:
            st.error(t("refresh_warn_title"))
            st.markdown(t("refresh_warn_body"))
            st.markdown(f"**{', '.join(known_live)}**")
            c1, c2 = st.columns(2)
            with c1:
                if st.button(t("stop_all_restart"), type="primary", key="btn_refresh_stop"):
                    for name in known_live:
                        _force_kill(name)
                    clean_orphans()
                    get_monitor_manager().reset()
                    st.session_state.refresh_dismissed = True
                    st.rerun()
            with c2:
                if st.button(t("dismiss"), key="btn_refresh_dismiss"):
                    st.session_state.refresh_dismissed = True
                    st.rerun()
            return  # Don't render the workflow until user decides

    # ── Row 1: Step (left) + Messages (right) ──
    step_col, msg_col = st.columns([3, 2])

    with step_col:
        @st.fragment(run_every=1)
        def _step_fragment():
            sync_action_lock()
            step_renderers = {
                0: render_step0,
                1: render_first_loop_sensors,
                2: render_first_loop_pgo,
                3: render_second_loop_mapping,
                4: render_step4,
            }
            renderer = step_renderers.get(st.session_state.current_step)
            if renderer:
                renderer()
        _step_fragment()

    with msg_col:
        @st.fragment(run_every=1)
        def _msg_fragment():
            render_messages()
        _msg_fragment()

    # ── Row 2: Session management ──
    st.divider()
    st.markdown(f"**{t('sessions')}**")
    selected = render_session_bar()

    # ── Row 3: Log viewer ──
    if selected:
        st.markdown(f"**{t('log')}**")
        _render_log_viewer(selected)


if __name__ == "__main__":
    main()
