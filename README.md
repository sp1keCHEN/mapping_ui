# ROS2 自动化建图工具用户手册 (Mapping UI)

这是一个交互式的 ROS2 建图辅助 Web 工具，旨在引导操作人员完成从雷达/IMU 启动、三维点云建图（faster-lio PGO）、原始数据包录制，到**在线**多楼层栅格地图生成（GridMapper）以及最终导航项目部署的完整流程。

---

## 快速开始

### 1. 环境准备与启动
在终端中进入项目目录，通过 `uv` 运行 Streamlit 应用：
```bash
cd ~/Workspace/algor_ws/src/mapping_UI
uv sync                     # 首次使用：自动同步并创建虚拟环境
uv run streamlit run app.py --server.port 8501
```

如果板卡或部署设备上的 workspace 路径不同，可以显式指定 `src` 目录：
```bash
MAPPING_UI_WS_SRC=/path/to/algor_ws/src uv run streamlit run app.py --server.port 8501
```

地图项目默认部署到 workspace 同级的 `~/Workspace/Maps/<项目名>/`。可通过 `MAPPING_UI_MAPS_ROOT` 改为其他目录，例如：
```bash
MAPPING_UI_MAPS_ROOT=/home/cat/Maps uv run streamlit run app.py --server.port 8501
```

应用启动时会优先从当前项目所在 workspace、`AMENT_PREFIX_PATH` / `COLCON_PREFIX_PATH` 的 ament index 中快速解析 `faster_lio`、`gridmapper`、`multi_map_nav` 等包路径；只有快速路径失败时才会有限扫描 `package.xml`。

## 双圈建图流程

页面将建图明确分成两圈，避免 PGO 回环优化影响在线栅格地图的坐标：

1. **第一圈（PGO + 排错 bag）**：启动 Livox、nav_bridge 和 `faster_lio ... pgo:=true`，录制原始雷达/IMU bag，完成一整圈后结束 PGO。
2. **PCD 审核**：页面以工程三视图显示 PGO 点云的主视（XZ）、左视（YZ）和俯视（XY），并显示点数与坐标范围。确认后可立即进入第二圈，也可保存先验、关闭节点，待充电后再恢复。
3. **第二圈（重定位 + 多地图）**：重新启动传感器，以 `relocal:=true prior_dir:=<第一圈 prior>` 启动 Faster-LIO，再启动 GridMapper；行走时通过页面切换楼层地图。本圈不录制 bag，也不运行 PGO。
4. **自动部署**：第二圈结束后，完整多地图输出与第一圈 PGO 会发布到 `~/Workspace/Maps/<项目名>/`。

### 2. 访问界面
在浏览器中打开：`http://localhost:8501`。
> [!TIP]
> 可以在侧边栏（Sidebar）最上方的下拉菜单中随时切换 **中文 (CN)** / **English (EN)** 界面。

---

## 界面布局说明

Web 界面采用三行式仪表盘布局，操作高度解耦，确保前台交互流畅：

1. **第一行：工作流步骤（左）与 实时消息（右）**
   - **左侧 (Step Workflow)**：核心引导步骤。展示当前步骤的控制按钮与状态（如等待话题频率）。
   - **右侧 (Messages)**：滚动显示系统执行历史与关键输出（如文件生成位置等）。该区域高度固定，支持滚轮向上查看历史，切换步骤时消息不会丢失。
2. **第二行：后台会话管理 (Sessions)**
   - 横向列出当前在后台**真实运行**的活跃后台会话。
   - 提供快捷键按钮：`[停止]`（停止当前会话）、`[重启]`（重新加载该会话）、`[刷新]`（刷新状态）、`[DL Log]`（下载完整文本日志文件）。
   - 选中某个会话后，会在下方自动以代码块形式显示它实际在后台执行的完整 Shell 命令。
3. **第三行：会话日志查看器 (Log)**
   - 实时（每秒自动刷新）展示在第二行中选中的活跃会话的终端最后 200 行输出，便于定位 ROS2 启动失败或运行报错。
4. **左侧侧边栏**
   - 提供流程进度指示器、文件路径概览（展开折叠面板可见），以及全程可用的红色 **“终止流程并关闭所有节点”** 按钮。

---

## 详细建图指导流程

### Step 0: 项目管理与环境检查
* 页面加载时自动列出当前 ROS2 环境变量及 Workspace 路径。
* “项目清单”汇总已部署的完整项目、已确认且可恢复第二圈的项目，以及仅保留第一圈先验的项目；完整项目会列出其楼层地图 ID。
* 选择“恢复已暂停的第二圈”后，系统会重新完成雷达、IMU、控制权和站立确认，再直接进入第二圈重定位；不会重复第一圈或重新录制 bag。
* 包路径会根据当前设备动态解析，页面文件路径面板会显示实际使用的 `PGO output`、`prior`、`gridmapper output` 和 `maps` 目录。
* **刷新防护说明**：如果您在建图途中意外刷新页面回到 Step 0，页面顶部会显示红色警告框，检测出后台有残留进程。您可以点击 **“停止所有并重置 (Stop All & Restart)”** 自动清空后台，或者点击 **“忽略 (Dismiss)”** 继续。
* 检查完毕后，点击 **"Start Workflow"** 进入第一步。

### Step 1: 传感器数据就绪检查
1. **Start Livox Lidar**：点击按钮启动激光雷达会话。系统会自动在后台以非阻塞方式获取 `/livox/lidar` 的频率，当有数据且频率正常时，显示实时 Hz 并打勾。
2. **Start nav_bridge**：点击按钮启动 IMU 节点，自动检查 `/imu/data` 频率。
   * 话题检测会先快速检查 publisher，再短时间采样 `ros2 topic hz`。这样可以避免 ROS2 CLI 卡住页面，同时仍然要求实际解析到 `average rate` 后才判定话题有效。
3. **Release Control**：当两个话题频率均正常后，点击释放底盘控制权。完毕后系统将自动推进到 Step 2。

### Step 2: 三维点云地图构建 (PGO SLAM)
1. **确认地图名称**：在文本框内定义本次建图的名称（默认为当前时间戳 `sensor_YYMMDD_HHMM`），然后点击 **"Confirm Map Name / 确认地图名称"**。
   * 地图名必须满足：1-48 个字符，只能包含英文字母、数字、下划线 `_`、短横线 `-`，且首字符必须是字母或数字。
   * 合法示例：`factory-A-01`、`company_floor1`、`sensor_260612_1530`。
   * 不建议/不允许使用空格、中文、斜杠、点号、引号、分号、`$()` 等字符。地图名会传递到 bag 名称、`prior` 目录、重定位参数、栅格地图文件名和导航 maps 目录。
2. **机器人站立**：操控机器人站立，随后点击 **"Robot is Standing"**。
3. **启动 SLAM**：点击启动 SLAM 节点。系统会在 30s 内自动检测 `/laser_mapping` 节点是否出现。
   * 启动新 SLAM 前，若 `PGO_output` 中已有旧的 `PGO.pcd` 或 `keyframes/`，系统会先将其归档到 `archive_YYYYMMDD_HHMMSS/`，避免误读上一轮输出。
4. **开始录制**：点击 **"Start Recording"** 启动 bag 包录制，记录雷达与 IMU 原始话题。
   * Bag 名称为 `<map_name>_sensor`，保存于 `~/bags/`。
5. **行走建图**：使用遥控器控制机器人平稳行走进行建图。
6. **结束建图**：行走完毕后，点击 **"Mapping Complete"**。系统将自动执行：
   * 停止 bag 录制。
   * 仅向 SLAM 节点发送一次 Ctrl+C 信号以触发 PGO 保存。
   * 自动等待 `PGO.pcd` 和 `keyframes/` 总体大小连续稳定，并优先等待 SLAM screen 自然退出。文件未稳定前不会主动强杀 `slam` screen，避免板卡上写盘较慢导致 `PGO.pcd` 未完整输出。
7. **审核与保存点云文件**：检测到 PGO 稳定就绪后，使用主视、左视和俯视确认点云质量。确认后可选择直接进入第二圈，或“结束本次作业”。
   * 选择结束本次作业时，系统会将点云与关键帧保存到 `prior/` 对应目录，并写入恢复检查点、关闭所有建图节点；充电后可从项目管理恢复第二圈。

### Step 2: 在线多楼层建图
1. **启动在线栅格建图**：SLAM 就绪后，点击 **"启动在线栅格建图"**。界面会归档之前的 `data/Output/multi_maps`，再启动 GridMapper。
2. **录制数据包**：GridMapper 启动后开始录制 `/livox/lidar` 与 `/imu/data` 原始数据，便于建图失败后离线排错；录包不再用于建图流程。
3. **切换楼层**：行走到楼梯、电梯、门或走廊入口时，在“多楼层地图切换”区确认目标 ID（默认依次为 `map_000`、`map_001`…）、通道类型和是否双向；UI 立即调用 `/switch_map`。
4. **结束与部署**：结束建图后，UI 停止录包并保存 GridMapper 与 PGO 输出，校验每张 `map_*.png/yaml`、`states/<map_id>_<width>x<height>c.gridmap.bin.gz`（兼容旧格式）、两个 CSV 后，原子发布到 `~/Workspace/Maps/<项目>/`。尺寸字段为栅格 cell 数；实际米数等于 cell 数乘以 resolution。无需重新编译导航包。

### Step 4: 流程结束与后台清理
* 界面会扫描当前是否还有残留的后台会话。
* 点击 **"Stop All Remaining Sessions"** 可一键清理干净。
* 点击 **"Reset Workflow"** 将重置主页面状态并返回 Step 0。

---

## 常见问题与排查指南

### 1. 地图名称会影响哪些文件？
假设确认的地图名为 `<map_name>`，系统会按以下规则传递：

| 用途 | 实际名称 / 路径 |
| --- | --- |
| Bag 录制目录 | `~/bags/<map_name>_sensor/` |
| PGO prior（流程中） | `faster-slam/prior/<map_name>/` |
| 最终定位 prior | `~/Maps/<map_name>/PGO.pcd`、`keyframes/` |
| 多楼层地图 | `~/Maps/<map_name>/map_000.*`、`map_001.*`… |
| 拓扑元数据 | `~/Maps/<map_name>/map_relations.csv`、`transition_points.csv` |

导航示例（无需编译或复制到 ROS package）：
```bash
ros2 launch multi_map_nav multi_map_nav.launch.py \
  multi_map_dir:=~/Maps/<map_name> initial_map:=map_000 use_sim_time:=true
```
重定位使用同一项目目录：`prior_dir:=~/Maps/<map_name>`。因此项目名必须保持简单稳定，避免特殊字符造成 shell 命令、ROS launch 参数或文件路径解析失败。

### 2. 启动 SLAM 或其它节点时提示 "laser_mapping not detected" 并超时闪退？
* **可能原因**：由于频繁启动和非正常退出，后台可能残留了双叉的 ROS2 孤儿进程，占满了同一个 DDS Domain 下的参与者席位（CycloneDDS 限制）。
* **排查方法**：在第二行选择异常的会话名（如 `slam`），在下方的日志查看器中查看具体崩溃原因。
* **解决办法**：点击侧边栏的 **"Abort Mapping"** 或者在第一步/第四步中选择“停止所有”按钮。这会调用系统底层的强力清理机制，强制清杀所有后台残留节点，释放 DDS 资源。

### 3. PGO.pcd 一直没有出现怎么办？
* **先看日志**：在会话管理中选择 `slam`，查看是否仍在做 PGO 优化或写文件。
* **不要手动强杀 slam screen**：系统在发送 Ctrl+C 后会等待文件稳定，过早杀掉 screen 可能导致 `PGO.pcd` 没有完整写出。
* **检查归档目录**：新一轮 SLAM 启动前，旧的 `PGO.pcd` 和 `keyframes/` 会被移动到 `PGO_output/archive_YYYYMMDD_HHMMSS/`，避免误用旧结果。

### 4. 我的手动调试会话或数据包播放被杀掉了？
* **放心运行**：应用内置的强力清理机制已排除了对包含 `ros2 bag` 命令行子进程的匹配。如果您在外部终端通过命令手动播放 bag，它不会被本 Web 界面误杀。

### 5. 如何在终端手动排查后台会话？
所有的后台进程均托管在独立的 `screen` 容器中运行，你可以通过终端直接操作：
```bash
screen -list            # 查看当前活跃的后台 screen 列表
screen -r slam          # 直接进入并实时查看 SLAM 进程终端
# 退出 screen 视图（保持后台运行）：在 screen 界面内按 Ctrl + A，然后按 D
```
