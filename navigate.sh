#!/bin/bash

# =====================================================
# 配置定义（编号 | 会话名 | 备注 | 启动命令 | 期望节点）
# =====================================================
load_config() {
    local mode=$1
    case "$mode" in
        "car+map")
            CONFIG=(
                "1 | tarkbot | 底盘驱动 | ros2 launch tarkbot_robot robot.launch.py pub_odom_tf:=false                                                                                                          | /tarkbot_robot"
                "2 | livox   | 雷达驱动 | ros2 launch livox_ros_driver2 msg_MID360_launch.py                                                                                                                    | /livox_lidar_publisher"
                "3 | slam    | 定位算法 | ros2 launch faster_lio slam.launch.py relocal:=true prior_dir:=company                                                                                                | /laser_mapping"
                "4 | terrain | 地形感知 | ros2 launch gridmapper local.launch.py rviz:=false                                                                                                                    | /gridmapper_node"
                "5 | local   | 局部规划 | ros2 launch local_planner local_planner.launch.py use_sim_time:=false start_rviz:=false debug_info:=true                                                              | /localPlanner /pathFollower"
                "6 | global  | 全局规划 | ros2 launch multi_map_nav multi_map_nav.launch.py initial_map:=company map_connections_file:=company use_fake_cmdvel:=true params_file:=new_local use_sim_time:=false | /planner_server /controller_server"
            )
            ;;
        "car-map")
            CONFIG=(
                "1 | tarkbot | 底盘驱动 | ros2 launch tarkbot_robot robot.launch.py pub_odom_tf:=false                                                                                                          | /tarkbot_robot"
                "2 | livox   | 雷达驱动 | ros2 launch livox_ros_driver2 msg_MID360_launch.py                                                                                                                    | /livox_lidar_publisher"
                "3 | slam    | 定位算法 | ros2 launch faster_lio slam.launch.py                                                                                                                                 | /laser_mapping"
                "4 | terrain | 地形感知 | ros2 launch gridmapper local.launch.py rviz:=false                                                                                                                    | /gridmapper_node"
                "5 | local   | 局部规划 | ros2 launch local_planner local_planner.launch.py use_sim_time:=false start_rviz:=false                                                                               | /localPlanner /pathFollower"
            )
            ;;
        "dog+map")
            CONFIG=(
                "2 | livox   | 雷达驱动 | ros2 launch livox_ros_driver2 msg_MID360_launch.py                                                                                                                    | /livox_lidar_publisher"
                "3 | slam    | 定位算法 | ros2 launch faster_lio slam.launch.py relocal:=true prior_dir:=company                                                                                                | /laser_mapping"
                "4 | terrain | 地形感知 | ros2 launch gridmapper local.launch.py rviz:=false                                                                                                                    | /gridmapper_node"
                "5 | local   | 局部规划 | ros2 launch local_planner local_planner.launch.py use_sim_time:=false start_rviz:=false                                                                               | /localPlanner /pathFollower"
                "6 | global  | 全局规划 | ros2 launch multi_map_nav multi_map_nav.launch.py initial_map:=company map_connections_file:=company use_fake_cmdvel:=true params_file:=new_local use_sim_time:=false | /planner_server /controller_server"
            )
            ;;
        "dog-map")
            CONFIG=(
                "2 | livox   | 雷达驱动 | ros2 launch livox_ros_driver2 msg_MID360_launch.py                                                                                                                    | /livox_lidar_publisher"
                "3 | slam    | 定位算法 | ros2 launch faster_lio slam.launch.py                                                                                                                                 | /laser_mapping"
                "4 | terrain | 地形感知 | ros2 launch gridmapper local.launch.py rviz:=false                                                                                                                    | /gridmapper_node"
                "5 | local   | 局部规划 | ros2 launch local_planner local_planner.launch.py use_sim_time:=false start_rviz:=false                                                                               | /localPlanner /pathFollower"
            )
            ;;
        *)
            echo -e "\033[31m错误: 无效的配置模式 '$mode'\033[0m"
            exit 1
            ;;
    esac
}

trim() {
    local var="$*"
    var="${var#"${var%%[![:space:]]*}"}"
    var="${var%"${var##*[![:space:]]}"}"
    echo -n "$var"
}

operate() {
    local action=$1
    local target_id=$2

    if [ "$action" == "stop" ]; then
        local v_list=()
        # 第一阶段：并发发送 Ctrl+C 指令
        while IFS='|' read -r id name desc cmd nodes; do
            id=$(trim "$id")
            name=$(trim "$name")
            nodes=$(trim "$nodes")
            if [[ "$target_id" == "all" || "$target_id" == "$id" ]]; then
                if screen -list | grep -q "\.${name}[[:space:]]"; then
                    echo "[$name] 发送 Ctrl+C (SIGINT)..."
                    screen -S "$name" -p 0 -X stuff "^C"
                    v_list+=("$name|$nodes")
                else
                    echo "[$name] 未运行，跳过验证"
                fi
            fi
        done < <(printf "%s\n" "${CONFIG[@]}" | sort -n)

        [ ${#v_list[@]} -eq 0 ] && return

        echo "等待 1 秒允许节点退出..."
        sleep 1

        # 第二阶段：再次并发发送 quit 指令
        for item in "${v_list[@]}"; do
            IFS='|' read -r v_name v_nodes <<< "$item"
            if screen -list | grep -q "\.${v_name}[[:space:]]"; then
                echo "[$v_name] 发送停止指令 (quit)..."
                screen -S "$v_name" -X quit
            fi
        done

        # 第三阶段：检测节点是否关闭
        for item in "${v_list[@]}"; do
            IFS='|' read -r v_name v_nodes <<< "$item"
            wait_nodes_gone "$v_name" "$v_nodes"
        done
        return
    fi

    # Start 逻辑：保持顺序启动并验证
    while IFS='|' read -r id name desc cmd nodes; do
        id=$(trim "$id")
        name=$(trim "$name")
        desc=$(trim "$desc")
        cmd=$(trim "$cmd")
        nodes=$(trim "$nodes")

        if [[ "$target_id" == "all" || "$target_id" == "$id" ]]; then
            if screen -list | grep -q "\.${name}[[:space:]]"; then
                echo "[$name] 已在运行"
            else
                echo "[$name] ($desc) 启动中..."
                screen -dmS "$name"
                screen -x -S "$name" -p 0 -X stuff "$cmd \n"
                wait_nodes "$name" "$nodes"
                sleep 1
            fi
        fi
    done < <(printf "%s\n" "${CONFIG[@]}" | sort -n)
}

show_menu() {
    printf "%s\n" "${CONFIG[@]}" | sort -n | while IFS='|' read -r id name desc cmd nodes; do
        printf "  \033[1;32m%s)\033[0m %-12s \033[36m# %s\033[0m\n" "$(trim "$id")" "$(trim "$name")" "$(trim "$desc")"
    done
    printf "  \033[1;32ma)\033[0m %-12s \033[36m# 启动/停止全部\033[0m\n" "all"
    printf "  \033[1;31mq)\033[0m %-12s \033[36m# 退出\033[0m\n" "quit"
    echo ""
}

wait_nodes() {
    local name=$1 nodes=$2 timeout=20
    echo -n "[$name] 验证节点启动..."
    for ((i=0; i<timeout; i++)); do
        local cur=$(ros2 node list 2>/dev/null)
        local ok=true
        for n in $nodes; do if ! echo "$cur" | grep -qx "$n" >/dev/null; then ok=false; break; fi; done
        if [ "$ok" = true ]; then echo -e "\033[32m 已就绪\033[0m"; return 0; fi
        sleep 0.5
    done
    echo -e "\033[31m 超时\033[0m"; return 1
}

wait_nodes_gone() {
    local name=$1 nodes=$2 timeout=60
    echo -n "[$name] 验证节点关闭..."
    for ((i=0; i<timeout; i++)); do
        local cur=$(ros2 node list 2>/dev/null)
        local any_exists=false
        for n in $nodes; do if echo "$cur" | grep -qx "$n" >/dev/null; then any_exists=true; break; fi; done
        if [ "$any_exists" = false ]; then echo -e "\033[32m 已关闭\033[0m"; return 0; fi
        sleep 0.5
    done
    echo -e "\033[31m 关闭超时\033[0m"; return 1
}

case "$1" in
    start|stop)
        # 检查是否输入了模式参数（第2个参数）
        if [ -z "$2" ]; then
            echo "用法：$0 [start|stop|status] [mode] [id]"
            echo "示例：$0 start car+map all"
            exit 1
        fi
        
        # 加载配置，不匹配会报错退出
        load_config "$2"

        # 判断是否输入了编号参数（第3个参数）
        if [ -n "$3" ]; then
            choice=$3
        else
            show_menu
            read -p "请输入指令 [ $1 ]: " choice
        fi
        
        case $choice in
            a|all) operate $1 "all" ;;
            q|quit|exit) exit 0 ;;
            *) operate $1 "$choice" ;;
        esac
        ;;
    status)
        screen -list
        ;;
    *)
        echo "用法：$0 [start|stop|status] [mode] [id]"
        echo "示例：$0 start car+map all"
        ;;
esac