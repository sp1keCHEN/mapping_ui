# Mapping Scripts

交互式 ROS2 建图 Web 工具，引导完成从传感器启动到栅格地图部署的全流程。

## 快速开始

```bash
cd ~/Workspace/algor_ws/src/mapping_scripts
uv sync                     # 首次：创建虚拟环境
uv run streamlit run app.py --server.port 8501
```

浏览器打开 `http://localhost:8501`。

## 流程概览

| Step | 内容 |
|------|------|
| **Step 0** 初始化 | 显示工作空间和 ROS2 信息，确认开始 |
| **Step 1** 传感器数据获取 | 启动 Livox 激光雷达、nav_bridge IMU，释放遥控器控制 |
| **Step 2** 三维点云地图构建 | 启动 PGO SLAM + Rviz，录制 bag，遥控行走建图，自动复制 prior |
| **Step 3** 栅格地图构建（离线） | 启动重定位 + gridmapper，回放 bag，自动重命名并部署地图文件 |
| **Step 4** 完成清理 | 停止残留传感器节点，一键重置工作流 |

## 界面布局

页面分为三栏：**侧边栏** | **左栏（Sessions）** | **右栏（Workflow）**

| 区域 | 内容 |
|------|------|
| **侧边栏** | 步骤进度、File Paths（折叠面板）、Abort Mapping（一键停止） |
| **左栏** | Session 下拉选择 → 状态 + 控制按钮（Stop / Restart / DL Log）→ 日志查看器（auto-refresh 开关）→ 所有 Session 运行状态概览 |
| **右栏** | 当前步骤消息面板、指令和操作按钮。流程线性推进，无需频繁切页 |

## 详细交互流程

### Step 0 初始化

打开页面后显示工作空间路径和 ROS2 版本信息，点击 **"Start Workflow"** 开始。

### Step 1 传感器数据获取

每个操作自动等待完成后进入下一个指令，无需手动 "Continue"：

1. **Start Livox Lidar** → 自动等待 `/livox/lidar` topic 有发布者（20s 超时），显示频率
2. **Start nav_bridge** → 自动等待 `/imu/data` topic 有发布者（20s 超时），显示频率
3. **Release Control** → 调用服务释放遥控器控制 → 自动进入 Step 2

### Step 2 三维点云地图构建

| # | 操作 | 说明 |
|---|------|------|
| 1 | 输入地图名 | 默认 `sensor_YYMMDD_HHMM` |
| 2 | 遥控站立 → 点击 "Robot is Standing" | |
| 3 | 点击 "Start SLAM + Rviz" | 自动等待 `laser_mapping` 节点出现（30s 超时） |
| 4 | 点击 "Start Recording" | 录制 `/livox/lidar` + `/imu/data` |
| 5 | 遥控行走建图 → 点击 "Mapping Complete" | 一键触发：停止 bag 录制 → SIGINT 停止 SLAM → 等待 PGO 输出 |
| 6 | 等待 PGO 文件生成 | 自动检测 `PGO.pcd` + `keyframes/` 出现且大小稳定（最长 120s） |
| 7 | 点击复制按钮 | `PGO.pcd` + `keyframes/` → `prior/<map_name>/` → 自动进入 Step 3 |

### Step 3 栅格地图构建（离线）

| # | 操作 | 说明 |
|---|------|------|
| 1 | 点击 "Start Relocalization" | `prior_dir=<map_name>`，自动等待 `laser_mapping` 节点 |
| 2 | 点击 "Start Grid Mapper" | 等待 Rviz 加载 → "Rviz Ready" |
| 3 | 点击 "Start Playback" | `ros2 bag play --clock`，自动检测播放完成，也可 "Skip Playback Wait" 跳过 |
| 4 | 观察栅格地图 → 点击 "Stop All Nodes" | |
| 5 | 检查输出文件 | `map.png / map.yaml / map_connections.txt`，标记 OK / MISSING |
| 6 | 点击 "Proceed to Rename" → "Rename" | 将 `map.*` 重命名为 `<map_name>.*`，更新 yaml 内 image 路径 |
| 7 | GIMP 编辑 → 点击 "Map Looks Good" | **不可修改分辨率** |
| 8 | 点击复制到 maps/ | 部署到 `multi_map_nav_ros2/maps/` |
| 9 | "Rebuild Now" 或 "Skip Rebuild" | 重新编译导航模块 |

### Step 4 完成清理

显示当前仍在运行的 session，点击 **"Stop All Remaining Sessions"** 一键停止。
点击 **"Reset Workflow"** 回到 Step 0 重新开始。

## 工作原理

所有后台 ROS2 进程运行在独立的 `screen` 会话中，通过 `tee` 将输出写入日志文件，UI 定期轮询读取。

- `ros2 node list` — 检查节点是否启动
- `ros2 topic info` — 检查 topic 是否有发布者
- `ros2 topic hz` — 显示数据频率
- bag 回放完成检测 — 进程退出自动检测，或手动跳过
- PGO 文件就绪检测 — 文件出现且大小连续 3 次检查不变才确认
- Launch 时同名 screen session 已存在会自动清理
- Session 注册表和 workflow 状态存储在 `st.session_state` 中，页面刷新不丢失

### Session 停止逻辑

参考 `navigate.sh` 的两阶段停止策略：

1. **Phase 1** — 通过 `screen -X stuff '^C'` 发送 Ctrl+C（SIGINT）
2. **Phase 2** — `screen -X quit` 终止仍存活的 session
3. **Phase 3** — `kill -9 {PID}` 强制杀死仍未退出的进程

screen 名称匹配使用 `.{name}[[:space:]]` 模式，避免同名前缀冲突（如 `livox` 不会匹配 `livox_backup`）。

## Screen 会话管理

左栏 Session 面板独立于 workflow，可随时查看/停止/重启任意 session：

| 会话名 | 对应节点 |
|--------|----------|
| `livox` | Livox 激光雷达 |
| `nav_bridge` | nav_bridge IMU |
| `slam` | PGO SLAM + Rviz |
| `bag_rec` | bag 录制 |
| `relocal` | 重定位节点 |
| `gridmapper` | Grid Mapper + Rviz |
| `bag_play` | bag 回放 |
| `build` | colcon 编译 |

Launch 时若同名 session 已存在，会自动清理后再启动。

独立调试：
```bash
screen -r slam          # attach 查看 SLAM 输出
screen -list            # 查看所有会话
```

## 文件路径

```
faster-slam/
├── data/PGO_output/          # PGO 建图原始输出
│   ├── PGO.pcd
│   └── keyframes/
├── prior/<map_name>/         # 建图完成后自动复制至此
│   ├── PGO.pcd
│   └── keyframes/

gridmapper/data/Output/       # 栅格地图生成输出（初始文件名）
├── map.png                   # → 重命名为 <map_name>.png
├── map.yaml                  # → 重命名为 <map_name>.yaml
└── map_connections.txt

multi_map_nav_ros2/maps/      # 导航模块调用的最终地图目录
```

## 注意事项

- Step 2 建图过程中确保机器人正面无动态物体，避免"鬼影"
- Step 3 中若用 GIMP 编辑栅格地图，**不可修改分辨率**
- 添加新地图文件到 `maps/` 后需重新编译 `multi_map_nav_ros2`
