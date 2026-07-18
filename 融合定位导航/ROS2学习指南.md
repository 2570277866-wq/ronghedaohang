# ROS2 学习指南 — 面向变电站巡检机器人开发

> 面向人群：本科科研入门，已有 Linux 基本操作能力
> 目标：从零搭建 ROS2 Humble 开发环境，跑通 SLAM + Nav2 自主导航全流程
> 预计时间：4~6 周（每天 2~3 小时）

---

## 目录

- [零、环境准备](#零环境准备)
- [一、ROS2 核心概念（第 1 周）](#一ros2-核心概念第-1-周)
- [二、Gazebo 仿真与机器人建模（第 2 周）](#二gazebo-仿真与机器人建模第-2-周)
- [三、SLAM 建图（第 3 周）](#三slam-建图第-3-周)
- [四、Nav2 自主导航（第 4 周）](#四nav2-自主导航第-4-周)
- [五、传感器融合进阶（第 5~6 周）](#五传感器融合进阶第-56-周)
- [六、变电站场景适配](#六变电站场景适配)
- [七、常用命令速查](#七常用命令速查)
- [八、常见问题排查](#八常见问题排查)
- [九、学习资源汇总](#九学习资源汇总)

---

## 零、环境准备

### 0.1 系统选择

| 方案 | 推荐度 | 说明 |
|------|:---:|------|
| **双系统 Ubuntu 22.04** | ⭐⭐⭐⭐⭐ | 性能最好，ROS2 Humble 官方支持 |
| VMware / VirtualBox 虚拟机 | ⭐⭐⭐⭐ | 方便快照回滚，但 Gazebo 仿真性能差 |
| WSL2 (Windows) | ⭐⭐⭐ | GUI 配置繁琐，Gazebo 需要额外折腾 |
| Docker | ⭐⭐⭐ | 环境可复现，但硬件直通配置复杂 |

> **强烈建议双系统**。虚拟机跑 Gazebo 仿真会卡到你怀疑人生。

### 0.2 安装 ROS2 Humble

```bash
# 1. 设置 locale 为 UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS2 仓库
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS2 Humble 桌面版
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# 4. 环境变量加入 .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 5. 验证安装
ros2 run demo_nodes_cpp talker     # 终端1
ros2 run demo_nodes_py listener    # 终端2 —— 能看到 "Hello World" 即成功
```

### 0.3 安装配套工具

```bash
# 开发工具
sudo apt install python3-pip python3-rosdep python3-vcstool
sudo rosdep init && rosdep update

# Gazebo 仿真
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# SLAM & 导航（核心）
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# 传感器融合
sudo apt install ros-humble-robot-localization

# TurtleBot3 仿真（入门学习用）
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo

# RViz2（可视化，一般随桌面版自带）
sudo apt install ros-humble-rviz2
```

### 0.4 创建工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

> `--symlink-install`：Python 文件修改后无需重新编译，调试神器。

---

## 一、ROS2 核心概念（第 1 周）

### 1.1 你必须搞懂的 6 个概念

| 概念 | 一句话 | 你的项目中对应什么 |
|------|--------|-------------------|
| **Node（节点）** | 一个执行单元，每个传感器/算法都是一个节点 | 激光雷达驱动节点、SLAM节点、Nav2节点 |
| **Topic（话题）** | 节点间"广播"数据的总线，发布/订阅模式 | `/scan`（激光数据）、`/odom`（里程计）、`/cmd_vel`（速度指令） |
| **Service（服务）** | 请求-应答模式，一次性调用 | 触发建图保存、切换导航模式 |
| **Action（动作）** | 带反馈的长时间任务 | "导航到充电桩"（可取消、可查看进度） |
| **Parameter（参数）** | 节点的配置项，运行时动态调整 | 激光雷达的扫描范围、机器人的最大速度 |
| **Launch File** | 同时启动多个节点的脚本文件 | 一键启动"激光驱动+SLAM+Nav2+RViz2" |

### 1.2 动手练习（第一天就敲命令）

```bash
# --- 理解 Topic ---
ros2 topic list                    # 列出所有话题
ros2 topic echo /topic_name        # 查看话题数据
ros2 topic info /topic_name        # 查看话题类型和发布者
ros2 topic hz /topic_name          # 查看话题发布频率

# --- 理解 Node ---
ros2 node list                     # 列出所有节点
ros2 node info /node_name          # 查看节点的发布/订阅/服务

# --- 理解 Service ---
ros2 service list                  # 列出所有服务
ros2 service call /service_name    # 手动调用服务

# --- 理解 Parameter ---
ros2 param list                    # 列出节点的所有参数
ros2 param get /node param_name    # 读取参数值
ros2 param set /node param_name value  # 修改参数值

# --- rqt_graph：可视化节点-话题关系图 ---
ros2 run rqt_graph rqt_graph       # 比纯文字直观 100 倍
```

### 1.3 最小上手项目：写一个发布者+订阅者

```bash
# 创建功能包
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_pkg

# 在 my_first_pkg/my_first_pkg/ 下创建 talker.py
```

**talker.py**（发布者）：
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.callback)
        self.count = 0

    def callback(self):
        msg = String()
        msg.data = f'Hello ROS2: {self.count}'
        self.pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    rclpy.spin(Talker())
    rclpy.shutdown()
```

**listener.py**（订阅者）：
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'chatter', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main():
    rclpy.init()
    rclpy.spin(Listener())
    rclpy.shutdown()
```

> 把这段代码跑通，你就理解了 ROS2 最基本的通信机制。你的项目本质上就是几千个这样的节点协同工作。

---

## 二、Gazebo 仿真与机器人建模（第 2 周）

### 2.1 为什么要先仿真？

- **省钱**：一个 16 线激光雷达 ¥3,000+，Gazebo 里免费
- **安全**：仿真里撞墙不会烧电机
- **可复现**：每次仿真环境一致，debug 不靠运气

### 2.2 TurtleBot3 — 入门首选

```bash
# 配置 TurtleBot3 模型
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc

# 启动仿真环境
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 新终端：键盘遥控
ros2 run turtlebot3_teleop teleop_keyboard

# 新终端：启动 RViz2 查看传感器
ros2 launch turtlebot3_bringup rviz2.launch.py
```

### 2.3 理解 TF（坐标变换）——这是很多人卡住的地方

```
map → odom → base_link → laser
                          → imu
                          → camera
```

- **map**：世界坐标系（全局原点）
- **odom**：里程计坐标系（会漂移，但连续）
- **base_link**：机器人本体中心
- **laser / imu / camera**：各传感器坐标系

```bash
# 查看 TF 树
ros2 run tf2_tools view_frames
# 生成 frames.pdf，可视化查看所有坐标关系

# 命令行查看 TF
ros2 run tf2_ros tf2_echo map base_link    # 查看两个坐标系间的关系
```

### 2.4 写一个简单的 URDF 描述自己的机器人

URDF（Unified Robot Description Format）就是机器人的"说明书"，描述了：
- 机器人各部件（link）的长宽高、质量
- 部件之间的连接方式（joint）
- 传感器安装位置

```xml
<!-- 最小 URDF 示例：一个带激光雷达的车体 -->
<?xml version="1.0"?>
<robot name="my_inspection_robot">
  <!-- 车体 -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.4 0.3 0.2"/></geometry>
      <material name="gray"/>
    </visual>
  </link>

  <!-- 激光雷达（固定在车体上方） -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_frame"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
  <link name="laser_frame"/>

  <!-- 给激光雷达加 Gazebo 插件，让它产生扫描数据 -->
  <gazebo reference="laser_frame">
    <sensor type="ray" name="lidar">
      <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>robot</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <ray>
          <scan><horizontal><samples>360</samples><resolution>1</resolution>
            <min_angle>-3.14</min_angle><max_angle>3.14</max_angle>
          </horizontal></scan>
          <range><min>0.1</min><max>30.0</max><resolution>0.01</resolution></range>
        </ray>
      </plugin>
    </sensor>
  </gazebo>
</robot>
```

> 有了 URDF，Gazebo 就能自动计算物理仿真；有了 TF，SLAM 和 Nav2 就知道传感器数据来自哪里。这两个搞定，后面的路就好走了。

---

## 三、SLAM 建图（第 3 周）

### 3.1 SLAM 在 ROS2 中的工作原理

```
激光雷达 → /scan ──┐
                   ├──→ SLAM Toolbox ──→ /map (地图)
里程计   → /odom ──┘                  ──→ /tf (map→odom 变换)
                                ──→ /map 话题（占据栅格地图）
```

### 3.2 用 SLAM Toolbox 建图

```bash
# 终端1：启动 Gazebo 仿真
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端2：启动 SLAM Toolbox（在线异步建图）
ros2 launch slam_toolbox online_async_launch.py

# 终端3：启动 RViz2 可视化
rviz2 -d /opt/ros/humble/share/slam_toolbox/rviz/async_launch.rviz

# 终端4：键盘控制机器人到处走，探索环境
ros2 run turtlebot3_teleop teleop_keyboard
```

> **关键**：让机器人缓慢而均匀地遍历整个环境，不要快速旋转。SLAM 需要相邻两帧激光扫描有足够重叠。

### 3.3 保存地图

```bash
# 保存地图（在 SLAM 运行过程中执行）
ros2 run nav2_map_server map_saver_cli -f ~/my_map

# 生成两个文件：
# my_map.yaml   — 地图元数据（分辨率、原点坐标）
# my_map.pgm    — 占据栅格图像（白色=空闲, 黑色=占据, 灰色=未知）
```

### 3.4 理解 SLAM Toolbox 的核心参数

| 参数 | 含义 | 建议值 |
|------|------|--------|
| `map_resolution` | 栅格分辨率(m) | 0.05 |
| `map_update_interval` | 地图更新间隔(s) | 1.0 |
| `max_laser_range` | 最大激光使用距离(m) | 15.0 |
| `minimum_travel_distance` | 最小移动距离才处理新数据(m) | 0.1 |
| `minimum_travel_heading` | 最小旋转角度才处理新数据(rad) | 0.1 |

---

## 四、Nav2 自主导航（第 4 周）

### 4.1 Nav2 架构全景

```
                    ┌─────────────┐
                    │  Behavior   │  行为树：任务编排
                    │    Tree     │  → "先去A点，拍照，再去B点，回充"
                    └──────┬──────┘
                           │
          ┌────────────────┼────────────────┐
          ▼                ▼                ▼
   ┌──────────┐    ┌──────────────┐   ┌──────────────┐
   │ Planner  │    │  Controller  │   │  Recoveries  │
   │  全局规划 │    │   局部控制    │   │   恢复行为    │
   │ A*/Dijk  │    │  DWA/TEB    │   │  旋转/后退    │
   └────┬─────┘    └──────┬───────┘   └──────────────┘
        │                 │
        ▼                 ▼
   ┌─────────────────────────────────────┐
   │            Costmap 代价地图           │
   │   ┌──────────┐  ┌──────────────┐     │
   │   │  Global   │  │    Local     │     │
   │   │  全局代价  │  │  局部代价    │     │
   │   └──────────┘  └──────────────┘     │
   └─────────────────────────────────────┘
```

**为什么需要两层代价地图？**
- **Global Costmap**：基于已有的静态地图（你刚建的 map.pgm），规划全局最优路径
- **Local Costmap**：基于实时传感器数据（激光雷达），发现动态障碍物（如走过的工人）

### 4.2 跑通 Nav2 完整流程

```bash
# 终端1：Gazebo + 机器人
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# 终端2：加载地图 + 启动 Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=true \
  map:=/path/to/my_map.yaml

# 终端3：RViz2 可视化 & 发送导航目标
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

> 在 RViz2 中点击 `2D Goal Pose` 按钮，在地图上点一个目标位置，机器人就会自主规划路径并行驶过去。

### 4.3 理解 Nav2 的四个核心组件

| 组件 | 功能 | 你项目中需要调什么 |
|------|------|-------------------|
| **planner_server** | 全局路径：从当前位置到目标的全局最优路径 | 变电站有"禁入区"，要调整代价地图的 lethal 区域 |
| **controller_server** | 局部控制：生成速度指令跟踪路径，同时避障 | 狭窄通道(80cm)需要调小 DWA 的膨胀半径 |
| **recoveries_server** | 卡住时自动恢复：原地旋转 → 后退 → 清除代价地图 | 金属反射导致错误障碍物时如何触发清除 |
| **bt_navigator** | 行为树：把巡检任务编排成"先去A→拍红外→去B→回充" | 巡检任务的编排逻辑 |

### 4.4 配置 Nav2 参数（以 DWA 为例）

```yaml
# nav2_params.yaml 关键参数
controller_server:
  ros__parameters:
    controller_frequency: 20.0          # 控制频率 Hz
    # DWA 参数
    max_vel_x: 0.5                      # 最大前进速度 m/s
    min_vel_x: -0.2                     # 最大后退速度
    max_vel_theta: 1.0                  # 最大旋转速度 rad/s
    # 代价地图
    inflation_radius: 0.3               # 障碍物膨胀半径（安全距离）
    # 目标容忍度
    xy_goal_tolerance: 0.05             # 到达目标的位置容忍度(m)
    yaw_goal_tolerance: 0.05            # 到达目标的角度容忍度(rad)
```

> **变电站场景注意**：膨胀半径不要太大（否则 80cm 通道过不去），但必须满足高压设备安全距离要求。

---

## 五、传感器融合进阶（第 5~6 周）

### 5.1 robot_localization：EKF 多传感器融合

```
LiDAR里程计 ──→ /lio_odom ──┐
IMU        ──→ /imu_data ──┼──→ EKF ──→ /odom (融合后的里程计)
轮式里程计  ──→ /wheel_odom ─┘         ──→ /tf (odom→base_link)
```

```bash
# 安装
sudo apt install ros-humble-robot-localization
```

**EKF 配置要点**：
- **频率最高的传感器做主时钟**（一般是 IMU，200Hz）
- 按传感器特性设置协方差矩阵（激光可靠→低协方差，视觉漂移→高协方差）
- 启动时机器人必须静止 2~3 秒，让 IMU 完成初始校准

### 5.2 FAST-LIO2：激光-惯性紧耦合

FAST-LIO2 是目前变电站场景下最推荐的 LiDAR+IMU SLAM 方案：

```bash
# 安装依赖
sudo apt install ros-humble-pcl-ros ros-humble-pcl-conversions
sudo apt install libpcl-dev libeigen3-dev

# 克隆编译
cd ~/ros2_ws/src
git clone https://github.com/hku-mars/FAST-LIO.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select fast_lio

# 运行（需要先配置好你激光雷达的驱动）
ros2 launch fast_lio mapping.launch.py
```

**为什么选 FAST-LIO2？**
- 速度快：迭代卡尔曼滤波，不是图优化，计算量小适合嵌入式计算平台
- 鲁棒：退化环境下（变电站长走廊、重复金属结构）仍能保持跟踪
- 抗 EMI：核心依赖 IMU（不受电磁干扰），LiDAR 做紧耦合校验

### 5.3 理解紧耦合 vs 松耦合（代码层面）

```
松耦合（EKF方式）：
  LiDAR ──→ 独立计算位姿 ──→ 位姿1 ──┐
  IMU   ──→ 独立积分位姿 ──→ 位姿2 ──┼──→ EKF加权平均 ──→ 最终位姿
  Wheel ──→ 独立推算位姿 ──→ 位姿3 ──┘
  
  问题：LiDAR 点云噪点增多时，算出的位姿本身就错了，融合也救不回来

紧耦合（FAST-LIO2方式）：
  LiDAR 原始点云 ──┐
  IMU 原始加速度 ──┼──→ 联合优化残差 ──→ 位姿
                    └── IMU 一直在提供先验，即使 LiDAR 暂时退化也不会丢失跟踪
```

---

## 六、变电站场景适配

### 6.1 仿变电站 Gazebo 环境搭建

```bash
# 思路：用 Gazebo 的 Building Editor 自己搭建
# 启动 Building Editor
gazebo

# 菜单：Edit → Building Editor
# 拖墙、放门、加柜体（模拟开关柜）
# 保存为 substation.world
```

**仿变电站场景要包含的元素**：
- 金属柜体（模拟开关柜、GIS 设备）
- 狭窄通道（80cm 宽）
- 长走廊（测试回环检测）
- 重复结构（测试地点识别鲁棒性）

### 6.2 模拟 GPS 拒止

在 Gazebo 中默认没有 GPS，这恰好匹配高压室场景。额外需要：
- 不加载 GPS 插件
- 在 Nav2 中不使用 `map→odom` 的外部矫正
- 纯靠 SLAM 定位

### 6.3 模拟传感器退化

```bash
# 给 Gazebo 中的激光雷达加噪声（模拟 EMI 噪点增多）
# 在 URDF 的激光雷达插件中添加：
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.02</stddev>  <!-- 正常值 0.005；EMI 下增大到 0.02 -->
</noise>
```

### 6.4 巡检任务的行为树

```xml
<!-- 一个简单的巡检行为树伪代码 -->
<root>
  <Sequence>
    <!-- 1. 导航到巡检点 A -->
    <NavigateToPose goal="inspection_point_A"/>
    <!-- 2. 执行巡检（拍照/测温） -->
    <ExecuteInspection duration="5s"/>
    <!-- 3. 导航到巡检点 B -->
    <NavigateToPose goal="inspection_point_B"/>
    <!-- 4. 执行巡检 -->
    <ExecuteInspection duration="3s"/>
    <!-- 5. 返回充电桩 -->
    <NavigateToPose goal="charging_station"/>
  </Sequence>
</root>
```

---

## 七、常用命令速查

### 7.1 工作空间

```bash
colcon build --symlink-install          # 编译
colcon build --packages-select pkg_name # 只编译某个包
colcon build --packages-up-to pkg_name  # 编译某包及其依赖

source ~/ros2_ws/install/setup.bash     # 加载工作空间环境
```

### 7.2 启动与调试

```bash
ros2 launch pkg_name launch_file.launch.py  # 启动 launch 文件
ros2 run pkg_name executable                # 运行单个节点
ros2 topic echo /topic_name                 # 查看话题实时数据
ros2 topic pub /topic_name msg_type "{data}" # 手动发一条消息
ros2 bag record /topic1 /topic2             # 录制数据包（离线调试用）
ros2 bag play bag_folder                    # 回放数据包
```

### 7.3 查看系统状态

```bash
ros2 topic list          # 所有话题
ros2 node list           # 所有节点
ros2 service list        # 所有服务
ros2 action list         # 所有动作
rqt_graph                # 可视化节点-话题关系图
ros2 run tf2_tools view_frames   # 生成 TF 树 PDF
```

### 7.4 日志与调试

```bash
ros2 run rqt_console rqt_console   # GUI 查看日志级别
ros2 topic echo /diagnostics       # 查看节点健康状态
```

---

## 八、常见问题排查

### 8.1 Gazebo 仿真中的高频问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 机器人不动，/cmd_vel 没有订阅者 | Nav2 未启动或话题名不匹配 | `ros2 topic info /cmd_vel` 查看订阅者 |
| 机器人打转，无法沿路径走 | DWA 的旋转速度权重过大 | 降低 `max_vel_theta`，增大路径跟随的横向误差权重 |
| 地图漂移，机器人"飞天" | TF 树断裂（某个 link 没有发布 TF） | `ros2 run tf2_tools view_frames` 检查 TF 连通性 |
| SLAM 建图出现"鬼影" | 激光雷达数据有噪声或机器人移动过快 | 降速，增大 `minimum_travel_distance` |

### 8.2 传感器融合常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| EKF 不收效（融合后反而更差） | 传感器协方差矩阵设置不合理 | 保守设置：激光=0.05, 视觉=0.1, 轮式=0.2 |
| IMU 偏置导致定位倾斜 | 启动时未静止，偏置校准不准确 | 启动前静止 3 秒，让 IMU 完成初始平差 |
| 里程计突然跳变 | 激光雷达因为金属反射"看到假墙" | 增大 LiDAR 最小距离阈值，视觉 SLAM 做二次校验 |

### 8.3 Nav2 导航常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 全局路径规划失败 | 目标点在代价地图的 lethal 区域 | 检查地图，确保目标区域是可通行的 |
| 机器人在障碍物前"犹豫" | 局部代价地图的膨胀半径过大 | 降低 `inflation_radius` |
| 恢复行为陷入死循环 | 清除代价地图后马上又被障碍物占据 | 增大恢复行为的旋转角度，尝试"换个方向看" |

---

## 九、学习资源汇总

### 9.1 官方文档（首选）

| 资源 | 链接 |
|------|------|
| ROS2 官方文档 | [docs.ros.org/en/humble](https://docs.ros.org/en/humble/) |
| Nav2 官方文档 | [navigation.ros.org](https://navigation.ros.org/) |
| Gazebo 官方教程 | [gazebosim.org/tutorials](https://gazebosim.org/tutorials) |
| SLAM Toolbox 文档 | [github.com/SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) |
| robot_localization 文档 | [github.com/cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) |

### 9.2 视频教程

| 资源 | 链接 |
|------|------|
| 古月居 ROS2 入门 21 讲 | B站搜索"古月居 ROS2" |
| 赵虚左 ROS2 实战 | B站搜索"赵虚左 ROS2" |
| Articulated Robotics ROS2 系列（英文） | [YouTube](https://www.youtube.com/@ArticulatedRobotics) |
| SLAM Course (Cyrill Stachniss) | [YouTube](https://www.youtube.com/playlist?list=PLgnQpQtFTOGQrZ4O5QzbIHgl3b1JHimN_) |

### 9.3 书籍

| 书名 | 作者 | 说明 |
|------|------|------|
| 《ROS2机器人开发实践》 | 胡春旭（古月居） | ROS2 实战入门首选 |
| 《视觉SLAM十四讲》(第2版) | 高翔等 | SLAM 理论圣经 |
| 《概率机器人》 | Thrun 等 | 状态估计理论基础 |

### 9.4 交流社区

| 社区 | 链接 |
|------|------|
| ROS 官方问答 | [robotics.stackexchange.com](https://robotics.stackexchange.com) |
| ROS 中文社区 | [fishros.org](https://fishros.org) (小鱼的一键安装脚本很好用) |
| GitHub Issues | 遇到 bug 先搜对应仓库的 Issues，99% 的问题别人已经提过 |

---

## 📅 4 周学习计划建议

```
第 1 周：ROS2 核心概念
  ├── Day 1-2：安装环境，跑通 talker/listener
  ├── Day 3-4：理解 Node/Topic/TF，rqt_graph 可视化
  ├── Day 5-6：写自己的 Publisher/Subscriber
  └── Day 7：学习 URDF，描述一个简单机器人

第 2 周：仿真入门
  ├── Day 1-2：TurtleBot3 Gazebo 仿真，键盘遥控
  ├── Day 3-4：理解 TF 树，传感器数据可视化
  ├── Day 5-6：URDF + Gazebo 搭建自己的机器人
  └── Day 7：在 Gazebo 中搭建简单环境

第 3 周：SLAM 建图
  ├── Day 1-2：SLAM Toolbox 建图，理解 /scan → /map
  ├── Day 3-4：参数调优，保存地图
  ├── Day 5-6：对比不同建图质量，理解回环检测
  └── Day 7：在自定义环境中建图

第 4 周：Nav2 自主导航
  ├── Day 1-2：加载地图，跑通 Nav2 基本导航
  ├── Day 3-4：理解 Planner/Controller/Costmap
  ├── Day 5-6：调参数（速度/膨胀半径/目标容忍度）
  └── Day 7：编写巡检行为树
```

---

> 💡 **最重要的建议**：不要试图在开始写代码前就理解所有理论。ROS2 是工程框架——**先跑起来，看到东西，再回头理解原理**。你第一天就能看到仿真机器人在 RViz2 里动起来，这种正向反馈是坚持下去的关键。
