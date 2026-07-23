"""User-facing translations for the multi-floor mapping workflow."""

import streamlit as st


TRANSLATIONS: dict[str, dict[str, str]] = {
    'page_title': {'en': 'Multi-Floor Mapping', 'zh': '多楼层建图'},
    'sidebar_title': {'en': 'Workflow', 'zh': '建图流程'},
    'file_paths': {'en': 'File Paths', 'zh': '文件路径'},
    'language': {'en': 'Language', 'zh': '语言'},
    'tbd': {'en': '(TBD)', 'zh': '（待定）'},
    'workspace_info': {'en': 'Workspace: `{workspace}`\nROS_DISTRO: `{distro}`',
                       'zh': '工作空间：`{workspace}`\nROS_DISTRO：`{distro}`'},
    'path_pgo_output': {'en': 'PGO output', 'zh': 'PGO 输出'},
    'path_prior': {'en': 'Prior', 'zh': 'Prior 点云'},
    'path_bag': {'en': 'Bag', 'zh': 'Bag'},
    'path_grid_map': {'en': 'Grid map', 'zh': '栅格地图'},
    'path_nav_maps': {'en': 'Nav maps', 'zh': '导航地图'},
    'abort_mapping': {'en': 'Terminate Workflow', 'zh': '终止建图流程'},
    'abort_done': {'en': 'Workflow terminated; all sessions stopped', 'zh': '建图流程已终止，所有会话均已停止'},
    'refresh_warn_title': {'en': '⚠️ Active Sessions Detected', 'zh': '⚠️ 检测到残留会话'},
    'refresh_warn_body': {'en': 'Background screen sessions from a previous workflow are still running. Refreshing the '
                          'page resets the UI but does **not** stop them.\n'
                          '\n'
                          'Please stop all sessions and restart the workflow.',
                          'zh': '上一次建图流程的后台 screen 会话仍在运行。刷新页面只会重置界面，**不会**自动停止它们。\n\n请先停止所有会话，再重新开始建图流程。'},
    'stop_all_restart': {'en': 'Stop All & Reset', 'zh': '停止所有 & 重置'},
    'dismiss': {'en': 'Dismiss', 'zh': '忽略'},
    'step0_name': {'en': 'Project Setup', 'zh': '项目配置'},
    'step1_name': {'en': 'System Readiness', 'zh': '系统就绪'},
    'step2_name': {'en': 'Prior Mapping', 'zh': '先验建图'},
    'step3_name': {'en': 'Multi-Floor Mapping', 'zh': '多层建图'},
    'step4_name': {'en': 'Project Delivery', 'zh': '成果交付'},
    's0_header': {'en': 'Project Setup', 'zh': '项目配置'},
    's0_setup_missing': {'en': 'install/setup.bash not found. Source it or rebuild first.',
                         'zh': '未找到 install/setup.bash，请先 source 或重新编译。'},
    's0_desc': {'en': 'This workflow uses two independent passes:\n'
                '\n'
                '1. **Pass 1 — PGO prior acquisition:** collect a loop-closed point cloud and a diagnostic ROS '
                'bag.\n'
                '2. **PCD review:** confirm coverage and loop closure, then save the prior.\n'
                '3. **Pass 2 — relocalized multi-floor mapping:** relocalize against the confirmed prior and create '
                'GridMapper floor maps online.\n'
                '4. **Validation and deployment:** publish the complete navigation project without rebuilding ROS '
                'packages.',
                'zh': '本流程由相互独立的两圈建图组成：\n'
                '\n'
                '1. **第一圈：PGO 定位先验采集**——生成回环优化点云并录制诊断 ROS bag。\n'
                '2. **PCD 审核**——确认覆盖范围和回环质量后保存定位先验。\n'
                '3. **第二圈：重定位多楼层建图**——基于已确认先验重定位，在线生成 GridMapper 楼层地图。\n'
                '4. **校验与部署**——发布完整导航地图项目，无需重新编译 ROS 软件包。'},
    'start_workflow': {'en': 'Start Workflow', 'zh': '开始建图'},
    's1_start_livox': {'en': 'Start Livox Lidar', 'zh': '启动激光雷达'},
    's1_wait_topic': {'en': 'Waiting for {topic} data... ({elapsed:.0f}s / 20s)',
                      'zh': '等待 {topic} 数据中… ({elapsed:.0f}s / 20s)'},
    's1_start_nav': {'en': 'Start nav_bridge', 'zh': '启动 nav_bridge'},
    's1_start_nav_desc': {'en': '**Start the nav_bridge IMU node.**', 'zh': '**启动 nav_bridge IMU 节点。**'},
    's1_release': {'en': 'Release Platform Control', 'zh': '释放底盘控制权'},
    's1_release_desc': {'en': 'Release platform control before operating the robot with the remote controller.',
                        'zh': '请先释放底盘控制权，再使用遥控器操控机器狗。'},
    's2_map_name': {'en': 'Map name', 'zh': '地图名称'},
    's2_confirm_name': {'en': 'Confirm Map Name', 'zh': '确认地图名称'},
    's2_name_invalid': {'en': 'Use 1-48 characters: letters, numbers, underscore, or hyphen. The first character must be '
                        'a letter or number.',
                        'zh': '请使用 1-48 个字符：字母、数字、下划线或短横线；首字符必须是字母或数字。'},
    's2_name_confirmed': {'en': 'Map name confirmed: {name}', 'zh': '地图名称已确认：{name}'},
    's2_stand_desc': {'en': '**Use the remote controller to stand up the robot.** Once standing, click below.',
                      'zh': '**使用遥控器控制机器人站立。** 站立后点击下方按钮。'},
    's2_stand_btn': {'en': 'Robot is Standing', 'zh': '机器人已站立'},
    's2_wait_pgo': {'en': 'Waiting for PGO output... ({elapsed:.0f}s / {seconds}s)',
                    'zh': '等待 PGO 输出... ({elapsed:.0f}s / {seconds}s)'},
    's2_slam_status': {'en': 'PGO SLAM', 'zh': 'PGO SLAM'},
    's2_bag_recording_status': {'en': 'Diagnostic ROS bag', 'zh': '诊断 ROS bag'},
    's4_header': {'en': 'Project Delivery', 'zh': '成果交付'},
    's4_stop_remaining': {'en': 'Stop All Remaining Sessions', 'zh': '停止所有残留会话'},
    's4_all_done': {'en': '**All mapping steps are done.**', 'zh': '**所有建图步骤已完成。**'},
    's4_project_location': {'en': 'Mapping project deployed to: `{path}`', 'zh': '建图项目已部署至：`{path}`'},
    's4_project_contents': {'en': 'This directory contains the PGO relocalization prior and the complete multi-floor '
                            'navigation map set.',
                            'zh': '该目录包含 PGO 重定位先验以及完整的多楼层导航地图。'},
    's4_reset': {'en': 'Reset Workflow', 'zh': '重置流程'},
    'sessions': {'en': 'Sessions', 'zh': '会话管理'},
    'select_session': {'en': 'Select session', 'zh': '选择会话'},
    'refresh': {'en': 'Refresh', 'zh': '刷新'},
    'stop': {'en': 'Stop', 'zh': '停止'},
    'restart': {'en': 'Restart', 'zh': '重启'},
    'log': {'en': 'Log', 'zh': '日志'},
    'no_active': {'en': 'No active screen sessions', 'zh': '无活跃 screen 会话'},
    'messages': {'en': 'Messages', 'zh': '消息'},
    'download_log': {'en': 'DL Log', 'zh': '下载日志'},
    'command': {'en': 'Command: `{cmd}`', 'zh': '命令：`{cmd}`'},
    'status_running': {'en': 'running', 'zh': '运行中'},
    'status_stopped': {'en': 'stopped', 'zh': '已停止'},
    'ok': {'en': 'OK', 'zh': '正常'},
    'msg_stopped': {'en': 'Stopped {name}', 'zh': '已停止 {name}'},
    'msg_restarted': {'en': 'Restarted {name}', 'zh': '已重启 {name}'},
    'msg_cannot_restart': {'en': 'Cannot restart: {name}', 'zh': '无法重启：{name}'},
    'msg_start_livox': {'en': 'Starting Livox lidar...', 'zh': '正在启动 Livox 激光雷达...'},
    'msg_start_nav': {'en': 'Starting nav_bridge for IMU...', 'zh': '正在启动 nav_bridge IMU...'},
    'msg_release_done': {'en': 'Control released: {output}', 'zh': '控制权已释放：{output}'},
    'msg_start_slam': {'en': 'Starting SLAM with PGO + Rviz...', 'zh': '正在启动带 PGO 的 SLAM...'},
    'msg_record_to': {'en': 'Recording to {path}/', 'zh': '正在录制到 {path}/'},
    'msg_bag_stopped': {'en': 'Bag recording stopped', 'zh': 'bag 录制已停止'},
    'msg_slam_sigint': {'en': 'Sent SIGINT to SLAM; waiting for PGO files to finish writing',
                        'zh': '已向 SLAM 发送 SIGINT，等待 PGO 文件写入完成'},
    'msg_pgo_ready': {'en': 'PGO output ready and stable', 'zh': 'PGO 输出已就绪且稳定'},
    'msg_pgo_timeout': {'en': 'WARN: PGO output not ready after {seconds}s', 'zh': '警告：{seconds}s 后 PGO 输出仍未就绪'},
    'msg_start_grid': {'en': 'Starting global grid mapper + Rviz...', 'zh': '正在启动全局栅格建图...'},
    'msg_context_recorder_started': {'en': 'Map context recorder started.', 'zh': '地图上下文记录器已启动。'},
    'msg_context_recorder_stopped': {'en': 'Map context recorder stopped and is writing metadata.', 'zh': '地图上下文记录器已停止，正在写入元数据。'},
    'msg_context_recorder_failed': {'en': 'WARN: map context recorder could not start: {error}', 'zh': '警告：地图上下文记录器启动失败：{error}'},
    'msg_send_grid_sigint': {'en': 'Sending SIGINT to gridmapper (saving map files)...',
                             'zh': '正在向 gridmapper 发送 SIGINT（保存地图文件）...'},
    'msg_grid_ready': {'en': 'Gridmapper output files ready', 'zh': 'gridmapper 输出文件已就绪'},
    'msg_copied_to': {'en': 'Copied {name} -> {dest}/', 'zh': '已复制 {name} -> {dest}/'},
    'msg_error_not_found': {'en': 'ERROR: {path} not found', 'zh': '错误：未找到 {path}'},
    'msg_archived_old': {'en': 'Archived old {name} -> {dest}/', 'zh': '已归档旧 {name} -> {dest}/'},
    's2_size_stability': {'en': 'Size stability: {count} / {target}', 'zh': '大小稳定性：{count} / {target}'},
    'not_found': {'en': '(not found)', 'zh': '（未找到）'},
    's3_wait_node': {'en': 'Waiting for `{node}` node... ({elapsed:.0f}s / {seconds}s)',
                     'zh': '等待 `{node}` 节点中... ({elapsed:.0f}s / {seconds}s)'},
    's3_relocal_status': {'en': 'Relocalization', 'zh': '重定位'},
    's3_grid_status': {'en': 'GridMapper', 'zh': 'GridMapper'},
    's3_multimap_header': {'en': 'Multi-floor map switching', 'zh': '多楼层地图切换'},
    's3_multimap_caption': {'en': 'Live GridMapper output: `{output}`', 'zh': 'GridMapper 实时输出：`{output}`'},
    's3_multimap_summary': {'en': 'Maps: {maps} · relations: {relations} · transitions: {transitions}',
                            'zh': '地图：{maps} · 关系：{relations} · 传送点：{transitions}'},
    's3_relations': {'en': 'Map relations', 'zh': '地图关系'},
    's3_transitions': {'en': 'Transition points', 'zh': '传送点'},
    'switch_history': {'en': 'Map switch service history', 'zh': '地图切换服务历史'},
    'last_switch_response': {'en': 'Latest complete service response', 'zh': '最近一次完整服务响应'},
    'switch_same_map': {'en': "Target map '{map}' is already active.", 'zh': '目标地图“{map}”已是当前活动地图。'},
    's3_target_map': {'en': 'Target map ID', 'zh': '目标地图 ID'},
    's3_transition_type': {'en': 'Transition type', 'zh': '通道类型'},
    's3_bidirectional': {'en': 'Bidirectional', 'zh': '双向通行'},
    's3_switch_map': {'en': 'Create / Switch Map', 'zh': '新建 / 切换地图'},
    's3_active_map': {'en': 'Active map: `{map}`', 'zh': '当前活动地图：`{map}`'},
    's3_archive_notice': {'en': 'Starting a new project archives the previous `{output}` output.',
                          'zh': '开始新项目会归档此前的 `{output}` 输出。'},
    'msg_multimap_archived': {'en': 'Archived previous multi-map output to {path}', 'zh': '已归档此前多地图输出到 {path}'},
    'msg_switch_ok': {'en': 'Switched active map to {target}', 'zh': '已切换活动地图至 {target}'},
    'msg_switch_failed': {'en': 'ERROR: map switch failed: {output}', 'zh': '错误：地图切换失败：{output}'},
    'msg_switch_not_ready': {'en': 'WARN: GridMapper has not received synchronized odometry yet; wait for the second-loop '
                             'input status to become ready.',
                             'zh': '警告：GridMapper 尚未收到同步里程计；请等待第二圈输入状态就绪后再切图。'},
    'msg_project_deployed': {'en': 'Mapping project deployed to {path}', 'zh': '建图项目已部署到 {path}'},
    'two_loop_first_sensors': {'en': 'System Readiness', 'zh': '系统就绪'},
    'two_loop_first_sensors_desc': {'en': 'Start the lidar and IMU bridge, then release platform control. These nodes '
                                    'remain active throughout both mapping passes and are stopped only after the '
                                    'final map review.',
                                    'zh': '启动激光雷达和 IMU 桥接后释放底盘控制权。它们将在两圈建图期间持续运行，直至最终地图审核完成后统一关闭。'},
    'two_loop_first_pgo': {'en': 'Prior Mapping', 'zh': '先验建图'},
    'two_loop_project_notice': {'en': 'This project name is shared by the first-loop PCD, debug bag, second-loop maps, '
                                'and final deployment.',
                                'zh': '项目名会贯穿第一圈 PCD、排错 bag、第二圈地图和最终部署目录。'},
    'two_loop_first_slam_desc': {'en': 'Start Faster-LIO with PGO. Complete one full loop so loop closure can optimize '
                                 'the PCD.',
                                 'zh': '启动带 PGO 的 Faster-LIO。请完整走一圈，使回环优化生成稳定 PCD。'},
    'two_loop_start_first_slam': {'en': 'Start Loop 1 PGO SLAM', 'zh': '启动第一圈 PGO SLAM'},
    'two_loop_first_drive_desc': {'en': 'Drive the first complete loop. Do not switch maps in this loop.',
                                  'zh': '完成第一圈行走。本圈不要切换地图。'},
    'two_loop_finish_first': {'en': 'Finish Loop 1 and Generate PCD', 'zh': '完成第一圈并生成 PCD'},
    'two_loop_pcd_review': {'en': 'PCD Quality Review', 'zh': 'PCD 质量审核'},
    'two_loop_pcd_points': {'en': 'Preview points', 'zh': '预览点数'},
    'two_loop_pcd_topdown': {'en': 'Top-down sampled PCD preview', 'zh': 'PCD 抽样俯视预览'},
    'two_loop_pcd_external': {'en': 'Browser preview unavailable ({error}). Verify `{path}` in RViz or an external '
                              'point-cloud tool.',
                              'zh': '网页无法预览（{error}）。请使用 RViz 或外部点云工具确认 `{path}`。'},
    'two_loop_pcd_confirm_desc': {'en': 'Confirm only after checking loop closure and coverage. Confirmation saves the '
                                  'prior; the lidar and IMU bridge remain active for Pass 2.',
                                  'zh': '请在确认回环和覆盖范围无误后继续。确认将保存定位先验；激光雷达和 IMU 桥接将保持运行，直接用于第二圈。'},
    'two_loop_confirm_pcd': {'en': 'PCD Approved — Save Prior and Start Pass 2', 'zh': '确认 PCD 无误 — 保存先验并开始第二圈'},
    'two_loop_second_header': {'en': 'Multi-Floor Mapping', 'zh': '多层建图'},
    'two_loop_start_relocal': {'en': 'Start Relocalized Faster-LIO', 'zh': '启动重定位 Faster-LIO'},
    'two_loop_start_relocal_desc': {'en': 'The confirmed PGO prior will be used for relocalization. GridMapper starts '
                                    'automatically after Faster-LIO is ready.',
                                    'zh': '将使用已确认的 PGO 定位先验启动重定位。Faster-LIO 就绪后，GridMapper 将自动启动。'},
    'two_loop_second_drive_desc': {'en': 'Drive the second loop and switch maps at stairs, doors, elevators, or '
                                   'corridors. No ROS bag is recorded in this loop.',
                                   'zh': '完成第二圈行走，在楼梯、门、电梯或走廊处切图。本圈不录制 ROS bag。'},
    'two_loop_finish_second': {'en': 'Finish Loop 2 and Deploy Maps', 'zh': '完成第二圈并部署地图'},
    'two_loop_recheck': {'en': 'Recheck Multi-Map Output', 'zh': '重新检查多地图输出'},
    'two_loop_maps_valid': {'en': 'Multi-floor map output is valid: {maps} ({relations} relations, {transitions} '
                            'transition points).',
                            'zh': '多楼层地图输出校验通过：{maps}（{relations} 条地图关系，{transitions} 个传送点）。'},
    'two_loop_context_ready': {'en': 'Map-context metadata is ready: `{path}`', 'zh': '地图上下文元数据已生成：`{path}`'},
    'two_loop_context_missing': {'en': 'Map-context metadata is missing: `{path}`. Mapping deployment can continue, but this project cannot provide reliable manual-inspection localization.',
                                 'zh': '未找到地图上下文元数据：`{path}`。可继续部署地图，但该项目无法提供可靠的人工巡检定位。'},
    'two_loop_preview_floor': {'en': 'Floor map to inspect', 'zh': '选择要检查的楼层地图'},
    'two_loop_preview_caption': {'en': '{map_id} occupancy map', 'zh': '{map_id} 栅格地图'},
    'two_loop_review_maps_desc': {'en': 'Inspect every floor map before deployment. After confirmation, the PGO prior and '
                                  'complete multi-floor map set will be published atomically to `{destination}`.',
                                  'zh': '请逐一检查各楼层栅格地图。确认后，PGO 定位先验和完整多楼层地图将原子发布到 `{destination}`。'},
    'two_loop_confirm_deploy': {'en': 'Confirm Maps and Deploy Project', 'zh': '确认地图并部署项目'},
    'two_loop_input_status': {'en': 'GridMapper input: cloud {cloud_hz:.1f} Hz · odometry {odom_hz:.1f} Hz',
                              'zh': 'GridMapper 输入：点云 {cloud_hz:.1f} Hz · 里程计 {odom_hz:.1f} Hz'},
    'two_loop_input_ready': {'en': 'Synchronized point-cloud and odometry input is active.', 'zh': '同步点云与里程计输入已激活。'},
    'workflow_progress': {'en': 'Stage {current} of {total}', 'zh': '当前阶段：{current} / {total}'},
    'two_loop_input_wait': {'en': 'Waiting for relocalized point cloud and odometry. Map switching is intentionally '
                            'unavailable.',
                            'zh': '正在等待重定位点云和里程计；在此之前已禁止切图。'},
    'two_loop_input_diagnose': {'en': 'If the rates remain 0, inspect the relocalization log. A failed initial '
                                'registration usually means the first-loop PCD has insufficient coverage or does '
                                'not match the current start area.',
                                'zh': '若频率持续为 0，请检查重定位日志。初始配准失败通常意味着第一圈 PCD 覆盖不足，或与第二圈起点环境不匹配。'},
    's4_remaining': {'en': '**Remaining running sessions:** {names}', 'zh': '**仍在运行的会话：** {names}'}}


def t(key: str, **kwargs) -> str:
    """Look up a translated string, formatting supplied placeholders."""
    entry = TRANSLATIONS.get(key)
    if entry is None:
        return key
    text = entry.get(st.session_state.get("lang", "zh"), entry["en"])
    return text.format(**kwargs) if kwargs else text
