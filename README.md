# Mapping Scripts

交互式 ROS2 建图工具，通过命令行引导完成从传感器启动到栅格地图部署的全流程。

## 快速开始

```bash
cd ~/Workspace/algor_ws/src/mapping_scripts
python3 slam_mapper.py
```

## 流程概览

| Step | 内容 | 自动 / 人工 |
|------|------|-------------|
| **Step 1** 传感器数据获取 | 启动 Livox 激光雷达、nav_bridge IMU，释放遥控器控制 | 自动启动 + 人工确认 |
| **Step 2** 三维点云地图构建 | 启动 PGO SLAM + Rviz，录制 bag，遥控行走建图，自动复制 prior | 自动启动 + 遥控建图 |
| **Step 3** 栅格地图构建（离线） | 启动重定位 + gridmapper，回放 bag，自动重命名并部署地图文件 | 自动回放 + 人工观察 |

## 详细交互流程

运行后打印 banner → `[Y/n] Ready to start ?` 确认后开始。

### Step 1 传感器数据获取

| # | 操作 | 类型 | 说明 |
|---|------|------|------|
| 1 | 启动 Livox 激光雷达 | 自动 | 等待 `/livox/lidar` topic 有发布者(20s)，打印频率 |
| 2 | 启动 nav_bridge IMU | 自动 | 等待 `/imu/data` topic 有发布者(20s)，打印频率 |
| 3 | `[Y/n] Call /nav_bridge_node/release_control ?` | 确认 | 调用服务释放遥控器控制，默认 Y |
| 4 | `—— Press Enter ——` | 暂停 | 确认传感器正常后继续 |

### Step 2 三维点云地图构建

| # | 操作 | 类型 | 说明 |
|---|------|------|------|
| 1 | `Map name [default: sensor_260520_1630]?` | 输入 | 地图名，回车使用默认 |
| 2 | `—— Press Enter ——` | 暂停 | **遥控机器狗站立**后确认 |
| 3 | 启动 PGO SLAM + Rviz | 自动 | 等待 `/faster_lio` 节点出现(30s) |
| 4 | 启动 bag 录制 | 自动 | 录制 `/livox/lidar` + `/imu/data` |
| 5 | `—— Press Enter ——` | 暂停 | **遥控行走建图**，观察 Rviz，完成后确认 |
| 6 | 停止 bag 录制 | 自动 | 打印 bag 保存路径 |
| 7 | 复制 PGO 输出 | 自动 | `PGO.pcd` + `keyframes/` → `prior/<map_name>/` |
| 8 | 停止 SLAM 节点 | 自动 | |
| 9 | `—— Press Enter ——` | 暂停 | 确认后进入 Step 3 |

### Step 3 栅格地图构建（离线）

| # | 操作 | 类型 | 说明 |
|---|------|------|------|
| 1 | 启动重定位节点 + gridmapper | 自动 | 使用 Step 2 生成的 prior |
| 2 | `—— Press Enter ——` | 暂停 | Rviz 加载后确认 |
| 3 | 回放 bag | 自动 | `ros2 bag play --clock` |
| 4 | `—— Press Enter ——` | 暂停 | **观察栅格地图效果**，完成后确认 |
| 5 | 停止所有节点 | 自动 | bag 回放、gridmapper、重定位 |
| 6 | 检查输出文件 | 自动 | 打印 `map.png` / `map.yaml` / `map_connections.txt` 的 `[OK] / [MISSING]` |
| 7 | `[Y/n] Rename map files to '<map_name>' ?` | 确认 | 重命名文件并更新 yaml 内 image 路径，默认 Y |
| 8 | `—— Press Enter ——` | 暂停 | **用 GIMP 编辑栅格地图**（不改分辨率），满意后确认 |
| 9 | `[Y/n] Copy map files to maps/ ?` | 确认 | 复制到 `multi_map_nav_ros2/maps/`，默认 Y |
| 10 | `[y/N] Rebuild multi_map_nav_ros2 ?` | 确认 | 重新编译导航模块，默认 N |
| 11 | `—— Press Enter ——` | 暂停 | 确认结束 |

**清理 →** 自动停止 Livox 和 nav_bridge 进程 → 结束。

### 交互符号说明

| 符号 | 含义 |
|------|------|
| `自动` | 脚本自动执行，无需操作 |
| `[Y/n]` | 确认询问，回车=默认Y，输入n=跳过 |
| `[y/N]` | 确认询问，回车=默认N，输入y=执行 |
| `—— Press Enter ——` | 暂停等待，完成人工操作后回车继续 |
| `输入` | 需要键入内容，回车使用默认值 |

## 工作原理

脚本通过 `subprocess.Popen` 后台启动各 ROS2 节点，在关键节点通过 `input()` 等待人工确认后继续。支持以下自动检测：

- `ros2 node list` — 检查节点是否启动
- `ros2 topic info` — 检查 topic 是否有发布者
- `ros2 topic hz` — 显示数据频率
- Ctrl+C 安全退出 — 自动清理所有后台进程

## 文件路径

```
faster_slam/
├── data/PGO_output/          # PGO 建图原始输出
│   ├── PGO.pcd
│   └── keyframes/
├── prior/<map_name>/         # 建图完成后自动复制至此
│   ├── PGO.pcd
│   └── keyframes/

gridmapper/data/Output/       # 栅格地图生成输出
├── <map_name>.png
├── <map_name>.yaml
└── map_connections.txt

multi_map_nav_ros2/maps/      # 导航模块调用的最终地图目录
```

## 注意事项

- Step 2 建图过程中确保机器人正面无动态物体，避免"鬼影"
- Step 3 中若用 GIMP 编辑栅格地图，**不可修改分辨率**
- 添加新地图文件到 `maps/` 后需重新编译 `multi_map_nav_ros2`
