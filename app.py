#!/usr/bin/env python3
"""Streamlit UI for interactive ROS2 mapping workflow.

Layout:
  - Sidebar: step progress, file paths, abort
  - Left panel: screen session management (independent of workflow)
  - Right panel: mapping workflow steps
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path

import streamlit as st

from i18n import TRANSLATIONS, t

# ---------------------------------------------------------------------------
# Page config
# ---------------------------------------------------------------------------
st.set_page_config(page_title="Mapping Scripts", layout="wide")

# Language selector — must be before any t() call
if "lang" not in st.session_state:
    st.session_state.lang = "en"

# ---------------------------------------------------------------------------
# Path constants
# ---------------------------------------------------------------------------
HOME = Path.home()
ALGOR_WS = HOME / "Workspace" / "algor_ws" / "src"
ALGOR_WS_ROOT = ALGOR_WS.parent
FASTER_SLAM = ALGOR_WS / "faster-slam"
PGO_OUTPUT = FASTER_SLAM / "data" / "PGO_output"
PRIOR_DIR = FASTER_SLAM / "prior"
GRIDMAPPER_OUTPUT = ALGOR_WS / "gridmapper" / "data" / "Output"
MAPS_DIR = ALGOR_WS / "multi_map_nav_ros2" / "maps"
BAGS_DIR = HOME / "bags"
LOGS_DIR = ALGOR_WS_ROOT / "mapping_logs"

# ROS2 launch commands
LIVOX_LAUNCH_CMD = "ros2 launch livox_ros_driver2 msg_multi_MID360_launch.py"
NAV_BRIDGE_LAUNCH_CMD = "ros2 launch nav_bridge nav_bridge.launch.py"
SLAM_PGO_LAUNCH_CMD = "ros2 launch faster_lio slam.launch.py pgo:=true rviz:=true"
RELOCAL_LAUNCH_CMD = "ros2 launch faster_lio slam.launch.py relocal:=true prior_dir:={prior}"
GRIDMAPPER_LAUNCH_CMD = "ros2 launch gridmapper global.launch.py rviz:=true"

LIVOX_TOPIC = "/livox/lidar"
IMU_TOPIC = "/imu/data"

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
    {"name": "bag_play", "label": "Bag Playback", "expected_nodes": []},
    {"name": "build", "label": "colcon Build", "expected_nodes": []},
]

SESSION_LABEL = {s["name"]: s["label"] for s in KNOWN_SESSIONS}
SESSION_NAMES = [s["name"] for s in KNOWN_SESSIONS]


def _init_sessions():
    if "sessions" not in st.session_state:
        st.session_state.sessions = {}


def _register_session(name: str, cmd: str, log_file: str):
    st.session_state.sessions[name] = {"cmd": cmd, "log_file": log_file}


def _get_session(name: str) -> dict | None:
    return st.session_state.sessions.get(name)


def _session_alive(name: str) -> bool:
    """Check if a screen session is running.

    Uses `.{name}[[:space:]]` pattern to avoid matching similar names
    (e.g. `livox` should not match `livox_backup`).
    """
    result = subprocess.run(
        f"screen -list | grep -q '\\.{name}[[:space:]]'",
        shell=True, capture_output=True,
    )
    return result.returncode == 0


def _screen_quit(name: str) -> None:
    """Send quit to a screen session."""
    try:
        subprocess.run(f"screen -S {name} -X quit", shell=True, timeout=5)
    except Exception:
        pass


def _force_kill(name: str) -> None:
    try:
        result = subprocess.run(
            f"screen -list | grep '\\.{name}[[:space:]]' | awk -F. '{{print $1}}'",
            shell=True, capture_output=True, text=True, timeout=5,
        )
        for line in result.stdout.strip().splitlines():
            pid = line.strip()
            if pid and pid.isdigit():
                try:
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
    except Exception:
        pass
    time.sleep(0.3)


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
    stdbuf_cmd = f"stdbuf -oL -eL {cmd} 2>&1 | tee -a {log_file}"
    if cwd:
        # cd first, then run the stdbuf-wrapped command
        full_cmd = f"cd {cwd} && {stdbuf_cmd}"
    else:
        full_cmd = stdbuf_cmd
    subprocess.run(
        f"screen -dmS {name} bash -c '{full_cmd}'",
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


def screen_stop_all() -> None:
    for name in list(st.session_state.sessions.keys()):
        _force_kill(name)


# ANSI escape sequence removal
_ANSI_RE = re.compile(r'\x1b\[[0-9;]*[a-zA-Z]|\x1b\]\d+[^a-z]*\x07')


def _strip_ansi(text: str) -> str:
    return _ANSI_RE.sub('', text)





def screen_read_log(name: str, max_lines: int = 100) -> str:
    info = _get_session(name)
    if not info:
        return "(session not registered)"
    try:
        result = subprocess.run(
            f"tail -n {max_lines} {info['log_file']}",
            shell=True, capture_output=True, text=True, timeout=3,
        )
        return _strip_ansi(result.stdout) or "(empty)"
    except FileNotFoundError:
        return "(log file not found)"
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
    try:
        result = subprocess.run(
            cmd, shell=True, env=ROS2_ENV,
            capture_output=True, text=True, timeout=timeout,
        )
        return result.stdout.strip()
    except subprocess.TimeoutExpired as e:
        if e.stdout:
            return e.stdout.decode('utf-8').strip() if isinstance(e.stdout, bytes) else e.stdout.strip()
        return None
    except Exception:
        return None


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
    output = run_ros2_cmd("ros2 node list")
    return output is not None and node_name in output


def get_topic_hz(topic: str, timeout: int = 3) -> float:
    """Get topic publish rate. Jazzy has no --timeout flag, so we rely
    on subprocess timeout to kill the process after *timeout* seconds."""
    output = run_ros2_cmd(
        f"ros2 topic hz {topic} --window 3",
        timeout=timeout,
    )
    if not output:
        return 0.0
    for line in output.splitlines():
        if "average rate:" in line:
            try:
                return float(line.split("average rate:")[1].strip().split()[0])
            except (ValueError, IndexError):
                pass
    return 0.0


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


def rename_grid_map(old_name: str, new_name: str) -> list[str]:
    messages = []
    for ext in (".png", ".yaml", ".txt"):
        old_path = GRIDMAPPER_OUTPUT / f"{old_name}{ext}"
        new_path = GRIDMAPPER_OUTPUT / f"{new_name}{ext}"
        if old_path.exists():
            if new_path.exists():
                new_path.unlink()
            old_path.rename(new_path)
            messages.append(f"Renamed {old_path.name} -> {new_path.name}")
    yaml_path = GRIDMAPPER_OUTPUT / f"{new_name}.yaml"
    if yaml_path.exists():
        content = yaml_path.read_text()
        # Replace whatever image filename is in the yaml with the new name
        fixed = re.sub(
            r"(image:\s*)[^\s#]+",
            r"\g<1>" + new_name + ".png",
            content,
        )
        if fixed != content:
            yaml_path.write_text(fixed)
            messages.append(f"Updated image path in {new_name}.yaml")
    return messages


def copy_grid_map(map_name: str) -> list[str]:
    messages = []
    MAPS_DIR.mkdir(parents=True, exist_ok=True)
    for ext in (".png", ".yaml"):
        src = GRIDMAPPER_OUTPUT / f"{map_name}{ext}"
        if src.exists():
            shutil.copy2(src, MAPS_DIR / src.name)
            messages.append(f"Copied {src.name} -> {MAPS_DIR}/")
    conn_src = GRIDMAPPER_OUTPUT / "map_connections.txt"
    if conn_src.exists():
        shutil.copy2(conn_src, MAPS_DIR / conn_src.name)
        messages.append(f"Copied map_connections.txt -> {MAPS_DIR}/")
    return messages


def copy_pgo_to_prior(map_name: str) -> list[str]:
    messages = []
    pgo_pcd = PGO_OUTPUT / "PGO.pcd"
    pgo_kf = PGO_OUTPUT / "keyframes"
    if not pgo_pcd.exists():
        return [f"ERROR: {pgo_pcd} not found"]
    if not pgo_kf.is_dir():
        return [f"ERROR: {pgo_kf} not found"]
    dest = PRIOR_DIR / map_name
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copy2(pgo_pcd, dest / "PGO.pcd")
    messages.append(f"Copied PGO.pcd -> {dest}/")
    if (dest / "keyframes").exists():
        shutil.rmtree(dest / "keyframes")
    shutil.copytree(pgo_kf, dest / "keyframes")
    messages.append(f"Copied keyframes/ -> {dest}/keyframes/")
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
        "step_messages": [],
        "step1_livox_hz": 0.0,
        "step1_imu_hz": 0.0,
        "pgo_last_size": -1,
        "pgo_stable_count": 0,
        "selected_session": KNOWN_SESSIONS[0]["name"],
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


def get_wait_start():
    return getattr(st.session_state, "wait_start", time.monotonic())


def clear_wait_state():
    if "wait_start" in st.session_state:
        del st.session_state.wait_start


def file_size_human(p: Path) -> str:
    size = p.stat().st_size
    if size < 1024:
        return f"{size} B"
    if size < 1024 * 1024:
        return f"{size / 1024:.1f} KB"
    return f"{size / 1024 / 1024:.1f} MB"


def step_status(step_id: int) -> str:
    if st.session_state.current_step > step_id:
        return "done"
    if st.session_state.current_step == step_id:
        return "running"
    return "not_started"


# ---------------------------------------------------------------------------
# Sidebar
# ---------------------------------------------------------------------------


def render_sidebar():
    st.sidebar.title(t("sidebar_title"))

    # Language toggle
    lang_options = {"en": "English", "zh": "中文"}
    current_lang = st.session_state.lang
    selected_lang = st.sidebar.radio(
        "🌐 Language",
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

    for sdef in _step_defs():
        sid = sdef["id"]
        status = step_status(sid)
        if status == "done":
            st.sidebar.markdown(f">&#10004; **Step {sid}**: {sdef['name']}")
        elif status == "running":
            st.sidebar.markdown(f">&#9654; **Step {sid}**: {sdef['name']}")
        else:
            st.sidebar.markdown(f"  Step {sid}: {sdef['name']}")

    st.sidebar.divider()

    with st.sidebar.expander(t("file_paths"), expanded=False):
        mn = st.session_state.map_name or "(TBD)"
        bd = st.session_state.bag_dir or "(TBD)"
        st.markdown(
            f"**PGO output:** `{PGO_OUTPUT}`\n\n"
            f"**Prior:** `{PRIOR_DIR}/{mn}`\n\n"
            f"**Bag:** `{bd}`\n\n"
            f"**Grid map:** `{GRIDMAPPER_OUTPUT}`\n\n"
            f"**Nav maps:** `{MAPS_DIR}`"
        )

    if 0 < st.session_state.current_step < 4:
        st.sidebar.divider()
        if st.sidebar.button(t("abort_mapping"), type="primary", key="sidebar_abort"):
            screen_stop_all()
            add_message(t("abort_done"))
            st.session_state.current_step = 0
            st.session_state.current_sub = "start"
            st.rerun()




# ---------------------------------------------------------------------------
# LEFT PANEL - Session management (independent of workflow)
# ---------------------------------------------------------------------------


def _list_screen_sessions() -> list[str]:
    """Return names of currently active screen sessions via `screen -ls`."""
    try:
        result = subprocess.run(
            "screen -ls", shell=True, capture_output=True, text=True, timeout=5,
        )
        names: list[str] = []
        for line in result.stdout.splitlines():
            line = line.strip()
            # Lines look like: "12345.slam	(05/29/26 17:00:00)	(Detached)"
            if "." in line and ("Detached" in line or "Attached" in line):
                # Extract the name after the PID dot
                part = line.split("\t")[0]   # "12345.slam"
                name = part.split(".", 1)[1] if "." in part else part
                names.append(name)
        return names
    except Exception:
        return []


@st.fragment(run_every=2)
def _render_log_viewer(session_name: str):
    """Log viewer as a Streamlit fragment — only this section re-renders."""
    st.subheader(t("log"))
    log_text = screen_read_log(session_name, max_lines=200)
    st.code(log_text, language="text", height=500)


def render_left_panel():
    st.header(t("sessions"))

    # Build session list: known sessions + any live ones not in the known list
    live_sessions = _list_screen_sessions()
    all_names = list(SESSION_NAMES)  # start with known
    for name in live_sessions:
        if name not in all_names:
            all_names.append(name)

    # Session selector
    default_idx = 0
    if st.session_state.selected_session in all_names:
        default_idx = all_names.index(st.session_state.selected_session)
    selected = st.selectbox(
        t("select_session"),
        options=all_names,
        index=default_idx,
        format_func=lambda n: SESSION_LABEL.get(n, n),
    )
    st.session_state.selected_session = selected

    # Live status
    alive = _session_alive(selected)
    info = _get_session(selected)

    status_icon = "&#9989;" if alive else "&#9760;"
    st.markdown(f"**{t('status')}:** {status_icon} {t('running') if alive else t('stopped')}")

    if info:
        st.caption(f"Command: `{info['cmd']}`")
        st.caption(f"Log: `{info['log_file']}`")
    else:
        st.caption(t("not_launched"))

    # Controls
    ctrl_cols = st.columns(3)
    with ctrl_cols[0]:
        if st.button(t("refresh"), key=f"left_refresh_{selected}", use_container_width=True):
            st.rerun()
    with ctrl_cols[1]:
        if alive:
            if st.button(t("stop"), key=f"left_stop_{selected}", type="primary", use_container_width=True):
                screen_stop(selected)
                add_message(f"Stopped {selected}")
                st.rerun()
        else:
            if st.button(t("restart"), key=f"left_restart_{selected}", use_container_width=True):
                result = screen_restart(selected)
                if result:
                    add_message(f"Restarted {selected}")
                else:
                    add_message(f"Unknown session: {selected}")
                st.rerun()
    with ctrl_cols[2]:
        if info and info["log_file"] and Path(info["log_file"]).exists():
            st.download_button(
                "DL Log",
                data=screen_read_log_full(selected),
                file_name=f"{selected}.log",
                mime="text/plain",
                key=f"left_dl_{selected}",
                use_container_width=True,
            )

    st.divider()
    _render_log_viewer(selected)

    # All sessions overview — live from screen -ls
    st.divider()
    st.subheader(t("all_sessions"))
    if live_sessions:
        for name in live_sessions:
            label = SESSION_LABEL.get(name, name)
            st.markdown(f"🟢 **{label}** (running)")
    else:
        st.caption(t("no_active"))

# ---------------------------------------------------------------------------
# RIGHT PANEL - Mapping workflow
# ---------------------------------------------------------------------------


def render_messages():
    messages = st.session_state.step_messages
    if not messages:
        return

    st.markdown(f"**{t('messages')}**")

    # Build message HTML
    lines: list[str] = []
    for msg in messages:
        if "ERROR" in msg:
            color = "#ff4b4b"
        elif "WARN" in msg:
            color = "#ffa726"
        elif any(kw in msg for kw in ("OK", "Copied", "Renamed", "ready")):
            color = "#66bb6a"
        else:
            color = "#ccc"
        escaped = msg.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        lines.append(f'<div style="color:{color};padding:2px 0;font-size:13px;font-family:monospace">{escaped}</div>')

    html_content = "\n".join(lines)
    # Fixed-height scrollable container that auto-scrolls to bottom
    st.components.v1.html(f"""
    <div id="msg-box" style="
        height: 200px;
        overflow-y: auto;
        background: #1a1a2e;
        border-radius: 6px;
        padding: 8px 12px;
        border: 1px solid #333;
    ">
        {html_content}
    </div>
    <script>
        var box = document.getElementById('msg-box');
        box.scrollTop = box.scrollHeight;
    </script>
    """, height=230)


def render_step0():
    st.header(t("s0_header"))

    setup_path = ALGOR_WS_ROOT / "install" / "setup.bash"
    ws_info = f"Workspace: `{ALGOR_WS_ROOT}`\nROS_DISTRO: `{ROS2_ENV.get('ROS_DISTRO', 'N/A')}`"
    if not setup_path.exists():
        st.warning(t("s0_setup_missing"))
    st.info(ws_info)

    st.markdown(t("s0_desc"))

    if st.button(t("start_workflow"), type="primary", key="btn_step0_start"):
        st.session_state.current_step = 1
        st.session_state.current_sub = "start_livox"
        st.session_state.step_messages = []
        st.rerun()


def render_step1():
    st.header(t("s1_header"))
    sub = st.session_state.current_sub

    # Start Livox
    if sub == "start_livox":
        st.markdown(t("s1_start_livox_desc"))
        if st.button(t("s1_start_livox"), type="primary", key="btn_s1_livox"):
            add_message("Starting Livox lidar...")
            screen_launch("livox", LIVOX_LAUNCH_CMD)
            st.session_state.current_sub = "wait_livox"
            st.session_state.wait_start = time.monotonic()
            st.rerun()

    if sub == "wait_livox":
        elapsed = time.monotonic() - get_wait_start()
        st.progress(min(elapsed / 20, 1.0))
        st.caption(f"Waiting for {LIVOX_TOPIC} data... ({elapsed:.0f}s / 20s)")

        hz = get_topic_hz(LIVOX_TOPIC)
        if hz > 0:
            st.session_state.step1_livox_hz = hz
            st.session_state.current_sub = "start_nav"
            clear_wait_state()
            add_message(f"{LIVOX_TOPIC} active (~{hz:.1f} Hz)")
            st.rerun()
        elif elapsed >= 20:
            st.session_state.current_sub = "start_nav"
            clear_wait_state()
            add_message(f"WARN: No data on {LIVOX_TOPIC} after 20s")
            st.rerun()
        else:
            time.sleep(2)
            st.rerun()

    # Livox status after launch
    if sub in ("start_nav", "wait_nav", "release_control"):
        alive = _session_alive("livox")
        hz_text = f"~{st.session_state.step1_livox_hz:.1f} Hz" if st.session_state.step1_livox_hz > 0 else ""
        st.markdown(f"**Livox:** {'running' if alive else 'stopped'} {hz_text}")

    # Start nav_bridge
    if sub == "start_nav":
        st.markdown(t("s1_start_nav_desc"))
        if st.button(t("s1_start_nav"), type="primary", key="btn_s1_nav"):
            add_message("Starting nav_bridge for IMU...")
            screen_launch("nav_bridge", NAV_BRIDGE_LAUNCH_CMD)
            st.session_state.current_sub = "wait_nav"
            st.session_state.wait_start = time.monotonic()
            st.rerun()

    if sub == "wait_nav":
        elapsed = time.monotonic() - get_wait_start()
        st.progress(min(elapsed / 20, 1.0))
        st.caption(f"Waiting for {IMU_TOPIC} data... ({elapsed:.0f}s / 20s)")

        hz = get_topic_hz(IMU_TOPIC)
        if hz > 0:
            st.session_state.step1_imu_hz = hz
            st.session_state.current_sub = "release_control"
            clear_wait_state()
            add_message(f"{IMU_TOPIC} active (~{hz:.1f} Hz)")
            st.rerun()
        elif elapsed >= 20:
            st.session_state.current_sub = "release_control"
            clear_wait_state()
            add_message(f"WARN: No data on {IMU_TOPIC} after 20s")
            st.rerun()
        else:
            time.sleep(2)
            st.rerun()

    # nav_bridge status
    if sub == "release_control":
        alive = _session_alive("nav_bridge")
        hz_text = f"~{st.session_state.step1_imu_hz:.1f} Hz" if st.session_state.step1_imu_hz > 0 else ""
        st.markdown(f"**nav_bridge:** {'running' if alive else 'stopped'} {hz_text}")

    # Release control
    if sub == "release_control":
        st.markdown(t("s1_release_desc"))
        if st.button(t("s1_release"), type="primary", key="btn_s1_release"):
            add_message("Calling /nav_bridge_node/release_control...")
            output = run_ros2_cmd(
                "ros2 service call /nav_bridge_node/release_control std_srvs/srv/Trigger"
            )
            add_message(f"Control released: {output or 'OK'}")
            st.session_state.current_step = 2
            st.session_state.current_sub = "stand_up"
            if not st.session_state.map_name:
                st.session_state.map_name = datetime.now().strftime("sensor_%y%m%d_%H%M")
            st.session_state.step_messages = []
            st.rerun()


def render_step2():
    st.header(t("s2_header"))
    sub = st.session_state.current_sub

    # Map name
    map_name = st.text_input(
        t("s2_map_name"), value=st.session_state.map_name, key="map_name_input",
    )
    st.session_state.map_name = map_name

    # Stand up
    if sub == "stand_up":
        st.markdown(t("s2_stand_desc"))
        if st.button(t("s2_stand_btn"), type="primary", key="btn_s2_stand"):
            add_message("Robot is standing")
            st.session_state.current_sub = "start_slam"
            st.rerun()

    # Start SLAM
    if sub == "start_slam":
        st.markdown(t("s2_start_slam_desc"))
        if st.button(t("s2_start_slam"), type="primary", key="btn_s2_slam"):
            add_message("Starting SLAM with PGO + Rviz...")
            screen_launch("slam", SLAM_PGO_LAUNCH_CMD)
            st.session_state.current_sub = "wait_slam"
            st.session_state.wait_start = time.monotonic()
            st.rerun()

    if sub == "wait_slam":
        elapsed = time.monotonic() - get_wait_start()
        st.progress(min(elapsed / 30, 1.0))
        st.caption(f"Waiting for `laser_mapping` node... ({elapsed:.0f}s / 30s)")

        if check_node_exists("laser_mapping"):
            st.session_state.current_sub = "start_bag"
            clear_wait_state()
            add_message("laser_mapping node is running")
            st.rerun()
        elif elapsed >= 30:
            st.session_state.current_sub = "start_bag"
            clear_wait_state()
            add_message("WARN: laser_mapping not detected after 30s")
            st.rerun()
        else:
            time.sleep(2)
            st.rerun()

    # SLAM status
    if sub in ("start_bag", "navigate"):
        alive = _session_alive("slam")
        st.markdown(f"**SLAM:** {'running' if alive else 'stopped'}")

    # Start bag recording
    if sub == "start_bag":
        bag_name = f"{map_name}_sensor"
        st.session_state.bag_name = bag_name
        st.markdown(
            f"**Start recording** `{LIVOX_TOPIC}` and `{IMU_TOPIC}` to bag `{bag_name}`."
        )
        if st.button(t("s2_start_rec"), type="primary", key="btn_s2_bag"):
            BAGS_DIR.mkdir(parents=True, exist_ok=True)
            add_message(f"Recording bag '{bag_name}' in ~/bags ...")
            cmd = f"ros2 bag record -o {bag_name} {LIVOX_TOPIC} {IMU_TOPIC}"
            screen_launch("bag_rec", cmd, cwd=str(BAGS_DIR))
            st.session_state.current_sub = "navigate"
            add_message(f"Recording to {BAGS_DIR}/{bag_name}/")
            st.rerun()

    # Navigate
    if sub == "navigate":
        rec_alive = _session_alive("bag_rec")
        slam_alive = _session_alive("slam")
        c1, c2 = st.columns(2)
        with c1:
            st.markdown(f"**SLAM:** {'running' if slam_alive else 'stopped'}")
        with c2:
            st.markdown(f"**Bag recording:** {'running' if rec_alive else 'stopped'}")

        st.markdown(t("s2_navigate_desc"))

        if st.button(t("s2_mapping_done"), type="primary", key="btn_s2_done"):
            add_message("Mapping complete - stopping bag recording...")
            screen_stop("bag_rec")
            add_message("Bag recording stopped")
            add_message("Stopping SLAM (SIGINT for PGO output)...")
            screen_stop("slam", graceful=True)
            st.session_state.current_sub = "wait_pgo"
            st.session_state.wait_start = time.monotonic()
            st.rerun()

    # Wait for PGO
    if sub == "wait_pgo":
        elapsed = time.monotonic() - get_wait_start()
        st.progress(min(elapsed / 120, 1.0))
        st.caption(f"Waiting for PGO output... ({elapsed:.0f}s / 120s)")

        pgo_pcd = PGO_OUTPUT / "PGO.pcd"
        pgo_kf = PGO_OUTPUT / "keyframes"
        pcd_exists = pgo_pcd.exists()
        kf_exists = pgo_kf.is_dir()

        c1, c2 = st.columns(2)
        with c1:
            if pcd_exists:
                st.markdown(f"**PGO.pcd:** OK ({file_size_human(pgo_pcd)})")
            else:
                st.markdown("**PGO.pcd:** waiting...")
        with c2:
            st.markdown(f"**keyframes/**: {'OK' if kf_exists else 'waiting...'}")

        if pcd_exists and kf_exists:
            cur_size = pgo_pcd.stat().st_size
            if cur_size == st.session_state.pgo_last_size and cur_size > 0:
                st.session_state.pgo_stable_count += 1
            elif cur_size > 0:
                st.session_state.pgo_last_size = cur_size
                st.session_state.pgo_stable_count = 0

            if st.session_state.pgo_stable_count >= 3:
                st.session_state.current_sub = "copy_pgo"
                clear_wait_state()
                st.session_state.pgo_last_size = -1
                st.session_state.pgo_stable_count = 0
                add_message("PGO output ready and stable")
                st.rerun()

            st.caption(f"Size stability: {st.session_state.pgo_stable_count} / 3")
            time.sleep(3)
            st.rerun()
        elif elapsed >= 120:
            if pcd_exists and kf_exists:
                add_message("WARN: PGO output found but size still changing")
            else:
                add_message("WARN: PGO output not ready after 120s")
            st.session_state.current_sub = "copy_pgo"
            clear_wait_state()
            st.rerun()

    # Copy PGO
    if sub == "copy_pgo":
        pgo_pcd = PGO_OUTPUT / "PGO.pcd"
        pgo_kf = PGO_OUTPUT / "keyframes"
        if pgo_pcd.exists() and pgo_kf.is_dir():
            st.markdown(
                f"**PGO output is ready.** Click to copy to `prior/{map_name}/`."
            )
            if st.button(f"Copy to prior/{map_name}/", type="primary", key="btn_s2_copy"):
                msgs = copy_pgo_to_prior(map_name)
                for m in msgs:
                    add_message(m)
                bag_dir = find_bag_dir(st.session_state.bag_name)
                st.session_state.bag_dir = bag_dir
                if bag_dir:
                    add_message(f"Bag saved at ./{bag_dir}")
                else:
                    add_message(f"WARN: Bag for '{st.session_state.bag_name}' not found")
                st.session_state.current_step = 3
                st.session_state.current_sub = "start_relocal"
                st.session_state.step_messages = []
                st.rerun()
        else:
            st.warning("PGO output not found. Check logs on the left.")
            if st.button("Continue to Step 3 (may fail)", key="btn_s2_no_pgo"):
                st.session_state.current_step = 3
                st.session_state.current_sub = "start_relocal"
                st.session_state.step_messages = []
                st.rerun()


def render_step3():
    st.header(t("s3_header"))
    sub = st.session_state.current_sub
    map_name = st.session_state.map_name
    bag_dir = st.session_state.bag_dir

    st.caption(f"Map: `{map_name}` | Bag: `{bag_dir or '(not found)'}`")

    # Start relocalization
    if sub == "start_relocal":
        st.markdown("**Stop live sensor nodes and start relocalization** with the prior map.")
        relocal_cmd = RELOCAL_LAUNCH_CMD.format(prior=map_name)
        st.code(relocal_cmd)
        if st.button("Start Relocalization", type="primary", key="btn_s3_relocal"):
            if _session_alive("livox"):
                screen_stop("livox")
                add_message("Stopped livox")
            if _session_alive("nav_bridge"):
                screen_stop("nav_bridge")
                add_message("Stopped nav_bridge")
            add_message(f"Starting relocalization with prior='{map_name}'...")
            screen_launch("relocal", relocal_cmd)
            st.session_state.current_sub = "wait_relocal"
            st.session_state.wait_start = time.monotonic()
            st.rerun()

    if sub == "wait_relocal":
        elapsed = time.monotonic() - get_wait_start()
        st.progress(min(elapsed / 30, 1.0))
        st.caption(f"Waiting for `laser_mapping` node... ({elapsed:.0f}s / 30s)")

        if check_node_exists("laser_mapping"):
            st.session_state.current_sub = "start_grid"
            clear_wait_state()
            add_message("laser_mapping node is running (relocal mode)")
            st.rerun()
        elif elapsed >= 30:
            st.session_state.current_sub = "start_grid"
            clear_wait_state()
            add_message("WARN: laser_mapping not detected after 30s")
            st.rerun()
        else:
            time.sleep(2)
            st.rerun()

    # Relocal status
    if sub in ("start_grid", "wait_rviz", "start_playback", "wait_playback"):
        alive = _session_alive("relocal")
        st.markdown(f"**Relocalization:** {'running' if alive else 'stopped'}")

    # Start grid mapper
    if sub == "start_grid":
        st.markdown("**Start the grid mapper + Rviz.**")
        if st.button("Start Grid Mapper", type="primary", key="btn_s3_grid"):
            add_message("Starting global grid mapper + Rviz...")
            screen_launch("gridmapper", GRIDMAPPER_LAUNCH_CMD)
            st.session_state.current_sub = "wait_rviz"
            st.rerun()

    if sub == "wait_rviz":
        alive = _session_alive("gridmapper")
        st.markdown(f"**Grid Mapper:** {'running' if alive else 'stopped'}")
        st.markdown("**Wait for Rviz to load**, then click below.")
        if st.button("Rviz Ready", type="primary", key="btn_s3_rviz"):
            st.session_state.current_sub = "start_playback"
            st.rerun()

    # Grid status
    if sub in ("start_playback", "wait_playback", "observe"):
        alive = _session_alive("gridmapper")
        st.markdown(f"**Grid Mapper:** {'running' if alive else 'stopped'}")

    # Start playback
    if sub == "start_playback":
        if bag_dir:
            st.markdown(f"**Play the recorded bag** with `--clock`.")
            st.code(f"ros2 bag play {bag_dir} --clock")
            if st.button("Start Playback", type="primary", key="btn_s3_play"):
                add_message(f"Playing bag '{bag_dir}' with --clock...")
                cmd = f"ros2 bag play {bag_dir} --clock"
                screen_launch("bag_play", cmd)
                st.session_state.current_sub = "wait_playback"
                st.rerun()
        else:
            st.warning("Bag directory not found.")
            st.markdown("**Manually run:** `ros2 bag play <your_bag>/ --clock`")
            if st.button("I've Started Playback Manually", type="primary", key="btn_s3_manual_play"):
                st.session_state.current_sub = "observe"
                st.rerun()

    # Wait playback
    if sub == "wait_playback":
        play_alive = _session_alive("bag_play")
        if play_alive:
            st.markdown("**Bag playback in progress...**")
            log_tail = screen_read_log("bag_play", max_lines=5)
            st.code(log_tail, language="text")
            if st.button("Skip Playback Wait", key="btn_s3_skip_play"):
                st.session_state.current_sub = "observe"
                st.rerun()
            time.sleep(3)
            st.rerun()
        else:
            add_message("Bag playback finished")
            st.session_state.current_sub = "observe"
            st.rerun()

    # Observe
    if sub == "observe":
        st.markdown(
            "**Check the grid map in Rviz.** "
            "When satisfied, click below to stop all nodes."
        )
        if st.button("Stop All Nodes", type="primary", key="btn_s3_stop"):
            st.session_state.current_sub = "stop_nodes"
            st.rerun()

    # Stop nodes
    if sub == "stop_nodes":
        st.markdown("**Stopping nodes...**")

        # Stop bag_play and relocal immediately (no file output needed)
        for name in ("bag_play", "relocal"):
            if _session_alive(name):
                screen_stop(name)
                add_message(f"Stopped {name}")

        # Gracefully stop gridmapper — send SIGINT and wait for it to save
        if _session_alive("gridmapper"):
            add_message("Sending SIGINT to gridmapper (saving map files)...")
            _send_ctrl_c("gridmapper")
        st.session_state.current_sub = "wait_grid_output"
        st.session_state.wait_start = time.monotonic()
        st.rerun()

    # Wait for gridmapper to finish saving
    if sub == "wait_grid_output":
        elapsed = time.monotonic() - get_wait_start()
        st.progress(min(elapsed / 30, 1.0))
        st.caption(f"Waiting for gridmapper to save map files... ({elapsed:.0f}s / 30s)")

        grid_alive = _session_alive("gridmapper")
        map_png = GRIDMAPPER_OUTPUT / "map.png"
        map_yaml = GRIDMAPPER_OUTPUT / "map.yaml"
        files_ready = map_png.exists() and map_yaml.exists()

        if files_ready:
            # Files appeared — clean up gridmapper if still running
            if grid_alive:
                screen_stop("gridmapper")
            clear_wait_state()
            add_message("Gridmapper output files ready")
            st.session_state.current_sub = "check_output"
            st.rerun()
        elif not grid_alive:
            # Gridmapper exited on its own — check if files appeared
            clear_wait_state()
            if files_ready:
                add_message("Gridmapper output files ready")
            else:
                add_message("WARN: Gridmapper exited but map files not found")
            st.session_state.current_sub = "check_output"
            st.rerun()
        elif elapsed >= 30:
            # Timeout — force stop and proceed
            add_message("WARN: Gridmapper save timeout (30s), force stopping")
            screen_stop("gridmapper")
            clear_wait_state()
            st.session_state.current_sub = "check_output"
            st.rerun()
        else:
            time.sleep(2)
            st.rerun()

    # Check output
    if sub == "check_output":
        st.markdown("**Check the generated map files:**")
        map_png = GRIDMAPPER_OUTPUT / "map.png"
        map_yaml = GRIDMAPPER_OUTPUT / "map.yaml"
        map_conn = GRIDMAPPER_OUTPUT / "map_connections.txt"
        for f in (map_png, map_yaml, map_conn):
            if f.exists():
                st.markdown(f"OK - `{f.name}` ({file_size_human(f)})")
            else:
                st.markdown(f"MISSING - `{f.name}`")
        if map_png.exists():
            st.image(str(map_png), caption="map.png (generated)", use_container_width=True)
            if st.button("Proceed to Rename", type="primary", key="btn_s3_rename_go"):
                st.session_state.current_sub = "rename"
                st.rerun()

    # Rename
    if sub == "rename":
        st.markdown(f"**Rename** `map.*` -> `{map_name}.*` and update yaml.")
        if st.button(f"Rename to '{map_name}'", type="primary", key="btn_s3_rename"):
            msgs = rename_grid_map("map", map_name)
            for m in msgs:
                add_message(m)
            st.session_state.current_sub = "review"
            st.rerun()

    # Review
    if sub == "review":
        renamed_png = GRIDMAPPER_OUTPUT / f"{map_name}.png"
        st.markdown(
            f"**Review the map** — open `{map_name}.png` in GIMP if needed.\n\n"
            "**Do NOT change the resolution.**"
        )
        if renamed_png.exists():
            st.image(str(renamed_png), caption=f"{map_name}.png", use_container_width=True)
        if st.button("Map Looks Good", type="primary", key="btn_s3_review"):
            st.session_state.current_sub = "copy_maps"
            st.rerun()

    # Copy to maps
    if sub == "copy_maps":
        st.markdown(f"**Copy** map files to `{MAPS_DIR}/` for navigation.")
        if st.button(f"Copy to {MAPS_DIR.name}/", type="primary", key="btn_s3_copy"):
            msgs = copy_grid_map(map_name)
            for m in msgs:
                add_message(m)
            st.session_state.current_sub = "rebuild"
            st.rerun()

    # Rebuild
    if sub == "rebuild":
        build_cmd = "colcon build --packages-select multi_map_nav --cmake-args -Wno-dev -DCMAKE_EXPORT_COMPILE_COMMANDS=1 --symlink-install"
        st.markdown(
            f"**Rebuild** the navigation module.\n\n"
            f"`{build_cmd}`"
        )
        c1, c2 = st.columns(2)
        with c1:
            if st.button("Rebuild Now", type="primary", key="btn_s3_rebuild"):
                add_message("Running colcon build...")
                cmd = f"{build_cmd} 2>&1"
                screen_launch("build", cmd, cwd=str(ALGOR_WS_ROOT))
                st.session_state.current_sub = "wait_build"
                st.rerun()
        with c2:
            if st.button("Skip Rebuild", key="btn_s3_skip_rebuild"):
                add_message("Skipped rebuild")
                st.session_state.current_step = 4
                st.session_state.current_sub = "cleanup"
                st.session_state.step_messages = []
                st.rerun()

    # Wait build
    if sub == "wait_build":
        build_alive = _session_alive("build")
        if build_alive:
            st.markdown("**Build in progress...**")
            log_tail = screen_read_log("build", max_lines=20)
            st.code(log_tail, language="text")
            time.sleep(3)
            st.rerun()
        else:
            add_message("Build complete")
            st.session_state.current_step = 4
            st.session_state.current_sub = "cleanup"
            st.session_state.step_messages = []
            st.rerun()


def render_step4():
    st.header(t("s4_header"))
    sub = st.session_state.current_sub

    if sub == "cleanup":
        sessions = list(st.session_state.sessions.items())
        running = [(n, i) for n, i in sessions if _session_alive(n)]

        if running:
            names = ", ".join(n for n, _ in running)
            st.markdown(f"**Remaining running sessions:** {names}")
            if st.button("Stop All Remaining Sessions", type="primary", key="btn_s4_stop"):
                for n, _ in running:
                    screen_stop(n)
                    add_message(f"Stopped {n}")
                st.session_state.current_sub = "done"
                st.rerun()
        else:
            st.session_state.current_sub = "done"
            st.rerun()

    if sub == "done":
        st.markdown(t("s4_all_done"))

        if st.button(t("s4_reset"), type="primary", key="btn_s4_reset"):
            screen_stop_all()
            if LOGS_DIR.exists():
                shutil.rmtree(LOGS_DIR, ignore_errors=True)
                LOGS_DIR.mkdir(parents=True, exist_ok=True)
            # Reset only workflow state, keep sessions registry
            workflow_keys = [
                "current_step", "current_sub", "map_name", "bag_name",
                "bag_dir", "step_messages", "step1_livox_hz", "step1_imu_hz",
                "pgo_last_size", "pgo_stable_count",
            ]
            for k in workflow_keys:
                if k in st.session_state:
                    del st.session_state[k]
            _init_state()
            st.rerun()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    st.title(t("page_title"))
    render_sidebar()

    # --- Browser-level refresh guard while workflow is active ---
    if 0 < st.session_state.current_step < 4:
        st.components.v1.html("""
        <script>
        window.addEventListener('beforeunload', function(e) {
            e.preventDefault();
            e.returnValue = '';
        });
        </script>
        """, height=0)

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
                    st.session_state.refresh_dismissed = True
                    st.rerun()
            with c2:
                if st.button(t("dismiss"), key="btn_refresh_dismiss"):
                    st.session_state.refresh_dismissed = True
                    st.rerun()
            return  # Don't render the workflow until user decides

    left_col, right_col = st.columns([1, 2])

    with left_col:
        render_left_panel()

    with right_col:
        step_placeholder = st.empty()
        with step_placeholder.container():
            step_renderers = {
                0: render_step0,
                1: render_step1,
                2: render_step2,
                3: render_step3,
                4: render_step4,
            }
            renderer = step_renderers.get(st.session_state.current_step)
            if renderer:
                renderer()

        st.divider()
        render_messages()


if __name__ == "__main__":
    main()
