"""Internationalization strings for the Mapping Scripts UI."""

import streamlit as st

TRANSLATIONS: dict[str, dict[str, str]] = {
    # ── Page / Sidebar ───────────────────────────────────────────────
    "page_title":           {"en": "Mapping Scripts",           "zh": "建图工具"},
    "sidebar_title":        {"en": "Mapping",                   "zh": "建图流程"},
    "file_paths":           {"en": "File Paths",                "zh": "文件路径"},
    "abort_mapping":        {"en": "Abort Mapping",             "zh": "终止建图"},
    "abort_done":           {"en": "ABORT: All sessions stopped", "zh": "终止：所有会话已停止"},

    # ── Refresh warning ──────────────────────────────────────────────
    "refresh_warn_title":   {"en": "⚠️ Active Sessions Detected",
                             "zh": "⚠️ 检测到残留会话"},
    "refresh_warn_body":    {"en": "Background screen sessions from a previous workflow are still running. "
                                   "Refreshing the page resets the UI but does **not** stop them.\n\n"
                                   "Please stop all sessions and restart the workflow.",
                             "zh": "上一次建图流程的后台 screen 会话仍在运行。"
                                   "刷新页面只会重置界面，**不会**自动停止它们。\n\n"
                                   "请先停止所有会话，再重新开始建图流程。"},
    "stop_all_restart":     {"en": "Stop All & Reset",          "zh": "停止所有 & 重置"},
    "dismiss":              {"en": "Dismiss",                   "zh": "忽略"},

    # ── Step names ───────────────────────────────────────────────────
    "step0_name":           {"en": "Init",                      "zh": "初始化"},
    "step1_name":           {"en": "Sensor Setup",              "zh": "传感器启动"},
    "step2_name":           {"en": "PGO SLAM",                  "zh": "三维建图"},
    "step3_name":           {"en": "Grid Map",                  "zh": "栅格地图"},
    "step4_name":           {"en": "Complete",                  "zh": "完成"},

    # ── Step 0 ───────────────────────────────────────────────────────
    "s0_header":            {"en": "Step 0: Initialization",    "zh": "步骤 0：初始化"},
    "s0_setup_missing":     {"en": "install/setup.bash not found. Source it or rebuild first.",
                             "zh": "未找到 install/setup.bash，请先 source 或重新编译。"},
    "s0_desc":              {"en": """This tool guides you through the full mapping workflow:

1. **Sensor Setup** — Start Livox Lidar and nav_bridge IMU
2. **PGO SLAM** — 3D pointcloud map construction
3. **Grid Map** — Offline grid map from bag playback
4. **Complete** — Cleanup and finish""",
                             "zh": """本工具引导你完成完整的建图流程：

1. **传感器启动** — 启动 Livox 激光雷达和 nav_bridge IMU
2. **三维建图** — 基于 PGO 的三维点云地图构建
3. **栅格地图** — 离线回放生成占据栅格地图
4. **完成** — 清理并结束"""},
    "start_workflow":       {"en": "Start Workflow",            "zh": "开始建图"},

    # ── Step 1 ───────────────────────────────────────────────────────
    "s1_header":            {"en": "Step 1: Sensor Data Acquisition",
                             "zh": "步骤 1：传感器数据获取"},
    "s1_start_livox":       {"en": "Start Livox Lidar",         "zh": "启动激光雷达"},
    "s1_start_livox_desc":  {"en": "**Start the Livox lidar node.**",
                             "zh": "**启动 Livox 激光雷达节点。**"},
    "s1_wait_topic":        {"en": "Waiting for {topic} data... ({elapsed:.0f}s / 20s)",
                             "zh": "等待 {topic} 数据中… ({elapsed:.0f}s / 20s)"},
    "s1_topic_active":      {"en": "{topic} active (~{hz:.1f} Hz)",
                             "zh": "{topic} 已激活（~{hz:.1f} Hz）"},
    "s1_topic_warn":        {"en": "WARN: No data on {topic} after 20s",
                             "zh": "警告：20 秒内未检测到 {topic} 数据"},
    "s1_start_nav":         {"en": "Start nav_bridge",          "zh": "启动 nav_bridge"},
    "s1_start_nav_desc":    {"en": "**Start the nav_bridge IMU node.**",
                             "zh": "**启动 nav_bridge IMU 节点。**"},
    "s1_release":           {"en": "Release Control",           "zh": "释放遥控权"},
    "s1_release_desc":      {"en": "**Release remote control so the robot accepts commands.**",
                             "zh": "**释放遥控器控制，以便遥控器操控建图。**"},

    # ── Step 2 ───────────────────────────────────────────────────────
    "s2_header":            {"en": "Step 2: PGO SLAM — 3D Pointcloud Map",
                             "zh": "步骤 2：PGO SLAM — 三维点云地图构建"},
    "s2_map_name":          {"en": "Map name",                  "zh": "地图名称"},
    "s2_stand_desc":        {"en": "**Use the remote controller to stand up the robot.** Once standing, click below.",
                             "zh": "**使用遥控器控制机器人站立。** 站立后点击下方按钮。"},
    "s2_stand_btn":         {"en": "Robot is Standing",         "zh": "机器人已站立"},
    "s2_start_slam":        {"en": "Start SLAM + Rviz",        "zh": "启动 SLAM + Rviz"},
    "s2_start_slam_desc":   {"en": "**Start the SLAM node with PGO + Rviz.**",
                             "zh": "**启动带回环检测 (PGO) 的 SLAM 节点和 Rviz。**"},
    "s2_start_rec":         {"en": "Start Recording",          "zh": "开始录制"},
    "s2_navigate_desc":     {"en": """**Drive the robot** through the scene with the remote controller.
- Watch Rviz for loop closures and map quality
- Keep dynamic objects out of the robot's frontal view""",
                             "zh": """**使用遥控器驱动机器人**在场景中运动。
- 注意观察 Rviz 中的回环和建图质量
- 确保机器人正面没有动态物体，避免"鬼影"效果"""},
    "s2_mapping_done":      {"en": "Mapping Complete",          "zh": "建图完成"},
    "s2_wait_pgo":          {"en": "Waiting for PGO output... ({elapsed:.0f}s / 120s)",
                             "zh": "等待 PGO 输出… ({elapsed:.0f}s / 120s)"},
    "s2_pgo_ready":         {"en": "PGO output ready and stable",
                             "zh": "PGO 输出已就绪且大小稳定"},
    "s2_copy_pgo":          {"en": "Copy to prior/{name}/",     "zh": "复制到 prior/{name}/"},

    # ── Step 3 ───────────────────────────────────────────────────────
    "s3_header":            {"en": "Step 3: Grid Map Construction (Offline)",
                             "zh": "步骤 3：栅格地图构建（离线）"},
    "s3_start_relocal":     {"en": "Start Relocalization",      "zh": "启动重定位"},
    "s3_start_grid":        {"en": "Start Grid Mapper",         "zh": "启动栅格建图"},
    "s3_rviz_ready":        {"en": "Rviz Ready",                "zh": "Rviz 已就绪"},
    "s3_start_play":        {"en": "Start Playback",            "zh": "开始回放"},
    "s3_skip_play":         {"en": "Skip Playback Wait",        "zh": "跳过回放等待"},
    "s3_observe_desc":      {"en": "**Check the grid map in Rviz.** When satisfied, click below to stop all nodes.",
                             "zh": "**在 Rviz 中检查栅格地图。** 确认满意后，点击下方停止所有节点。"},
    "s3_stop_all":          {"en": "Stop All Nodes",            "zh": "停止所有节点"},
    "s3_wait_grid":         {"en": "Waiting for gridmapper to save map files... ({elapsed:.0f}s / 30s)",
                             "zh": "等待 gridmapper 保存地图文件… ({elapsed:.0f}s / 30s)"},
    "s3_check_output":      {"en": "**Check the generated map files:**",
                             "zh": "**检查生成的地图文件：**"},
    "s3_proceed_rename":    {"en": "Proceed to Rename",         "zh": "继续重命名"},
    "s3_rename_btn":        {"en": "Rename to '{name}'",        "zh": "重命名为 '{name}'"},
    "s3_review_desc":       {"en": "**Review the map** — open `{name}.png` in GIMP if needed.\n\n**Do NOT change the resolution.**",
                             "zh": "**检查地图** — 如需修改可用 GIMP 打开 `{name}.png`。\n\n**不可修改分辨率！**"},
    "s3_map_good":          {"en": "Map Looks Good",            "zh": "地图没问题"},
    "s3_copy_maps":         {"en": "Copy to {name}/",           "zh": "复制到 {name}/"},
    "s3_rebuild_desc":      {"en": "**Rebuild** the navigation module.",
                             "zh": "**重新编译**导航模块。"},
    "s3_rebuild_now":       {"en": "Rebuild Now",               "zh": "立即编译"},
    "s3_skip_rebuild":      {"en": "Skip Rebuild",              "zh": "跳过编译"},

    # ── Step 4 ───────────────────────────────────────────────────────
    "s4_header":            {"en": "Workflow Complete",         "zh": "建图流程完成"},
    "s4_stop_remaining":    {"en": "Stop All Remaining Sessions",
                             "zh": "停止所有残留会话"},
    "s4_all_done":          {"en": "**All mapping steps are done.**",
                             "zh": "**所有建图步骤已完成。**"},
    "s4_reset":             {"en": "Reset Workflow",            "zh": "重置流程"},

    # ── Left panel ───────────────────────────────────────────────────
    "sessions":             {"en": "Sessions",                  "zh": "会话管理"},
    "select_session":       {"en": "Select session",            "zh": "选择会话"},
    "status":               {"en": "Status",                    "zh": "状态"},
    "running":              {"en": "running",                   "zh": "运行中"},
    "stopped":              {"en": "stopped",                   "zh": "已停止"},
    "not_launched":         {"en": "Not yet launched",          "zh": "尚未启动"},
    "refresh":              {"en": "Refresh",                   "zh": "刷新"},
    "stop":                 {"en": "Stop",                      "zh": "停止"},
    "restart":              {"en": "Restart",                   "zh": "重启"},
    "log":                  {"en": "Log",                       "zh": "日志"},
    "all_sessions":         {"en": "All Sessions",              "zh": "所有会话"},
    "no_active":            {"en": "No active screen sessions", "zh": "无活跃 screen 会话"},
    "messages":             {"en": "Messages",                  "zh": "消息"},
}


def t(key: str, **kwargs) -> str:
    """Look up a translated string by key, with optional .format() kwargs."""
    lang = st.session_state.get("lang", "en")
    entry = TRANSLATIONS.get(key)
    if entry is None:
        return key
    text = entry.get(lang, entry.get("en", key))
    if kwargs:
        text = text.format(**kwargs)
    return text
