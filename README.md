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

## 详细交互流程

### Step 0 初始化

打开页面后显示工作空间路径和 ROS2 版本信息，点击 **"Start Workflow"** 开始。

### Step 1 传感器数据获取

| # | 操作 | 说明 |
|---|------|------|
| 1 | 点击 "Start Livox Lidar" | 自动等待 `/livox/lidar` topic 有发布者(20s)，显示频率 |
| 2 | 点击 "Start nav_bridge" | 自动等待 `/imu/data` topic 有发布者(20s)，显示频率 |
| 3 | 点击 "Release Control" | 调用服务释放遥控器控制 |
| 4 | 点击 "Continue to Step 2" | 确认传感器正常后进入下一步 |

### Step 2 三维点云地图构建

| # | 操作 | 说明 |
|---|------|------|
| 1 | 输入地图名 | 默认 `sensor_YYMMDD_HHMM` |
| 2 | 遥控站立 → 点击 "Robot is Standing" | |
| 3 | 点击 "Start SLAM + Rviz" | 自动等待 `laser_mapping` 节点出现(30s) |
| 4 | 点击 "Start Recording" | 录制 `/livox/lidar` + `/imu/data` |
| 5 | 遥控行走建图 → 点击 "Mapping Complete" | 观察 Rviz 回环 |
| 6 | 点击 "Confirm Stop Bag" → "Confirm Stop SLAM" | 分两步确认，SLAM 收到 SIGINT 后输出 PGO 结果 |
| 7 | 等待 PGO 文件生成 | 自动检测 `PGO.pcd` + `keyframes/` 出现且大小稳定(最长120s) |
| 8 | 点击复制按钮 | `PGO.pcd` + `keyframes/` → `prior/<map_name>/` |
| 9 | 点击 "Continue to Step 3" | 进入栅格地图构建 |

### Step 3 栅格地图构建（离线）

| # | 操作 | 说明 |
|---|------|------|
| 1 | 点击 "Start Relocalization" | `prior_dir=<map_name>`，等待 `laser_mapping` 节点 |
| 2 | 点击 "Start Grid Mapper" | |
| 3 | Rviz 加载 → 点击 "Rviz Ready" | |
| 4 | 点击 "Start Playback" | `ros2 bag play --clock`，自动检测播放完成，也可手动跳过 |
| 5 | 观察栅格地图 → 点击 "Stop All Nodes" | |
| 6 | 检查输出文件 | gridmapper 初始输出为 `map.png / map.yaml / map_connections.txt`，页面标记 `[OK] / [MISSING]` |
| 7 | 点击 "Proceed to Rename" → "Rename" | 将 `map.*` 重命名为 `<map_name>.*`，更新 yaml 内 image 路径 |
| 8 | GIMP 编辑 → 点击 "Map Looks Good" | **不可修改分辨率** |
| 9 | 点击复制到 maps/ | 部署到 `multi_map_nav_ros2/maps/` |
| 10 | "Rebuild Now" 或 "Skip Rebuild" | 重新编译导航模块 |

### Step 4 完成清理

显示当前仍在运行的 session，点击 **"Stop All Remaining Sessions"** 一键停止。
点击 **"Reset Workflow"** 可回到 Step 0 重新开始。

## 工作原理

所有后台 ROS2 进程运行在独立的 `screen` 会话中，通过 `tee` 将输出写入日志文件，UI 定期轮询读取。

- `ros2 node list` — 检查节点是否启动
- `ros2 topic info` — 检查 topic 是否有发布者
- `ros2 topic hz` — 显示数据频率
- bag 回放完成检测 — 进程退出自动检测，或手动 "Skip to Next" 跳过
- PGO 文件就绪检测 — 文件出现且大小连续 3 次检查不变才确认
- Launch 时同名 screen session 已存在会自动清理，不会重名

## 界面布局

| 区域 | 内容 |
|------|------|
| **侧边栏** | 工作流步骤进度、Session 管理面板（Stop All / Log / Stop / Restart / Refresh）、**Abort Mapping**（一键停止所有节点并回到 Step 0）、实时日志查看器 |
| **主区域** | Step Messages 时间戳消息面板（可折叠）、当前步骤详情和操作按钮 |
| **底部** | **File Paths** 折叠面板（显示所有读写路径）、Live Log 折叠面板（Refresh / Download Log） |

## Screen 会话管理

每个后台进程分配一个命名 screen 会话，可在侧边栏直接管理：

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
