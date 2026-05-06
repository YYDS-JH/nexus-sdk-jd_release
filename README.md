# Nexus SDK — 发布版

本仓库为 **Nexus-Arm v05 → AR5 遥操作系统**的发布包，包含预编译的 ROS 2 功能包、系统配置工具和 Docker 环境镜像。

## 目录结构

```
nexus-sdk/
├── src/                         # ROS 2 功能包（预编译，含静态库）
│   ├── infra_msg/               # 自定义消息/服务定义
│   ├── teleop_adapter/          # 遥操作设备适配器（Nexus-Arm / AR5）
│   ├── human_data/              # 运动学解算节点
│   ├── robot_controller/        # 机械臂控制器节点
│   └── nexus_manage/            # 系统管理与启动编排
├── tool/                        # 宿主机系统配置工具
│   ├── set_rules.sh             # 安装 Nexus-Arm 串口 udev 规则
│   ├── nexus_arm_tty.rules      # udev 规则文件
│   ├── setup_pedal_permissions.sh  # 配置脚踏板/键盘输入权限
│   └── PEDAL_PERMISSIONS.md     # 输入权限说明
└── docker/
    └── x86_64/
        └── nexus-sdk-env-x86_64.docker.tar.gz  # 预构建 Docker 环境镜像
```

## 系统架构

```
  ┌─────────────────────────────────────────────────────────┐
  │                   Master 机（Nexus-Arm v05）             │
  │                                                         │
  │  nexus_arm_adapter ──▶ human_data_solver ──▶ manager   │
  │                                  │              │       │
  │                        master_controller        │       │
  └──────────────────────────────────┼──────────────┼───────┘
                                     │  ROS 2 DDS   │
                                     │ (CycloneDDS) │
  ┌──────────────────────────────────┼──────────────┼───────┐
  │                   Slave 机（AR5 机械臂）         │       │
  │                                  │              │       │
  │  ar5_arm_adapter ◀── human_data_solver ◀────────┘       │
  │                            │                            │
  │                   slave_controller                      │
  └─────────────────────────────────────────────────────────┘
```

---

## 一、克隆仓库

```bash
git clone -b jd_release git@gitlab.pegasus-ai.cn:infra-embedded-software/nexus-sdk.git ~/ws
cd ~/ws
```

## x86 实时内核离线分发与安装（6.8.2-rt11）

如果需要把当前开发机的实时内核分发到其他 x86 机器，可直接使用仓库内目录：

`kernel/x86-rt-kernel-6.8.2-rt11/`

目录内容：

- `boot/`：内核启动文件（`vmlinuz`、`initrd`、`System.map`、`config`）
- `modules/lib-modules-6.8.2-rt11.tar.gz`：对应内核模块
- `scripts/install_rt_kernel.sh`：目标机安装脚本
- `kernel-packages.txt`：当前机内核包版本记录

在目标 x86 机器安装：

```bash
# 1) 从当前仓库拷贝该目录到目标机（示例）
scp -r kernel/x86-rt-kernel-6.8.2-rt11 <user>@<target-ip>:~/

# 2) 在目标机执行安装
cd ~/x86-rt-kernel-6.8.2-rt11
./scripts/install_rt_kernel.sh

# 3) 重启并验证
sudo reboot
uname -r
```

预期输出：`6.8.2-rt11`。

> 注意：目标机需为 x86_64 且 ABI 兼容；若启用 Secure Boot，需先关闭或完成内核签名后再使用该内核。

---

## 二、宿主机系统准备（`tool/` 工具）

> **两台机器（Master 和 Slave）均需执行以下步骤。**

### 2.1 安装 Nexus-Arm 串口 udev 规则（Master 机必做）

Nexus-Arm v05 通过 USB 串口连接，需要安装 udev 规则以在 `/dev/nexus_arm` 创建稳定设备链接：

```bash
cd ~/ws/tool
sudo ./set_rules.sh
```

验证规则生效（连接 Nexus-Arm 后）：

```bash
ls -la /dev/nexus_arm   # 应显示符号链接
```

> **Docker 用户**：udev 规则仅在宿主机运行，容器内通过 `-v /dev:/dev` 挂载后可直接访问 `/dev/nexus_arm`。

### 2.2 配置输入设备权限（Master 机必做）

系统通过 `libevdev` 直接读取脚踏板/键盘的 `/dev/input/event*` 设备，需要将当前用户加入 `input` 组：

```bash
cd ~/ws/tool
sudo ./setup_pedal_permissions.sh
```

执行后**注销并重新登录**使权限生效，或在当前终端执行：

```bash
newgrp input
```

验证权限：

```bash
ls -la /dev/input/event*   # 应有 input 组读取权限
```

---

## 三、非 Docker 环境

### 3.1 依赖安装

**操作系统要求**：Ubuntu 22.04

#### 安装 ROS 2 Humble

如尚未安装，参考 [ROS 2 Humble 官方文档](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)：

```bash
sudo apt update && sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

#### 安装系统依赖

```bash
sudo apt update && sudo apt install -y \
    build-essential \
    gcc \
    libeigen3-dev \
    libevdev-dev \
    ros-humble-pinocchio \
    ros-humble-rmw-cyclonedds-cpp
```

> `gcc` 用于 launch 首次运行时按需编译 DDS 日志过滤库（自动完成，无需手动操作）。

#### 安装 OSQP 和 OsqpEigen（robot_controller 依赖）

`robot_controller` 使用 OSQP 求解器，需从源码编译安装：

```bash
# 编译安装 OSQP
cd ~
git clone --branch release-0.6.3 --recursive https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/osqp/install ..
make -j$(nproc) && make install
cd ~

# 编译安装 OsqpEigen
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=~/osqp-eigen/install \
      -DCMAKE_PREFIX_PATH=~/osqp/install ..
make -j$(nproc) && make install
cd ~
```

### 3.2 编译工作空间

```bash
source /opt/ros/humble/setup.bash
cd ~/ws
colcon build --symlink-install
source install/setup.bash
```

> 如果 OSQP 未安装到默认路径，编译前需设置环境变量：
> ```bash
> export OSQP_INSTALL_DIR=~/osqp/install
> export OSQP_EIGEN_INSTALL_DIR=~/osqp-eigen/install
> ```

### 3.3 启动遥操作系统

编译完成后，每次打开新终端都需要先 source 环境：

```bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
```

#### 方式 A：单机模式（两台机器的所有节点运行在同一台机器上，仅用于调试）

```bash
ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py
```

#### 方式 B：分布式模式（推荐，Master 和 Slave 分别在各自机器上运行）

**Master 机**（运行 Nexus-Arm v05 侧节点）：

```bash
ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py \
    role:=master \
    network_interface:=<Master机网卡名或IP>
```

**Slave 机**（运行 AR5 侧节点）：

```bash
ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py \
    role:=slave \
    network_interface:=<Slave机网卡名或IP>
```

#### launch 参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `role` | `""` | `master`：仅启动主臂侧；`slave`：仅启动从臂侧；空字符串：全部启动 |
| `domain_id` | `7` | ROS_DOMAIN_ID，同一系统内所有机器必须一致 |
| `network_interface` | `auto` | 用于 DDS 跨机通信的网卡名或 IP。多网卡时必须显式指定，例如 `enp3s0` 或 `10.18.20.25`。查看可用网卡：`ip -4 addr \| grep "10.18"` |

#### 网络配置说明

系统使用静态 Peer 发现（禁用多播），两台机器的 IP 已内置于 launch 文件：

| 角色 | 机器 | IP |
|------|------|----|
| Master | Nexus-Arm v05 机 | `10.18.64.46` |
| Slave | AR5 机 | `10.18.20.25` |

如机器 IP 发生变化，需修改 `src/nexus_manage/launch/nexus_arm_v05_to_ar5_real_system.launch.py` 中 `<Peers>` 块的地址，重新编译后生效。

#### 硬件连接配置

AR5 机械臂 IP 和本机 IP 配置在从臂适配器配置文件中：

```bash
# 文件路径（编译后）：
~/ws/install/teleop_adapter/share/teleop_adapter/config/slave_ar5_single.yaml
```

关键参数：

```yaml
slave_arm_adapter:
  ros__parameters:
    right_arm_port: '192.168.0.160'    # AR5 控制器 IP
    right_arm_local_ip: '192.168.0.23' # 本机（Slave 机）IP
```

Nexus-Arm v05 串口设备路径配置：

```bash
# 文件路径：
~/ws/install/teleop_adapter/share/teleop_adapter/config/master_nexus_single.yaml
```

```yaml
master_nexus_adapter:
  ros__parameters:
    right_arm_port: '/dev/nexus_arm'   # udev 规则创建的符号链接
```

---

## 四、Docker 环境

Docker 镜像已预装 ROS 2 Humble 及全部系统依赖，无需手动安装。

### 4.1 加载 Docker 镜像

```bash
cd ~/ws/docker/x86_64
docker load -i nexus-sdk-env-x86_64.docker.tar.gz
```

查看加载后的镜像名称：

```bash
docker images | grep nexus
```

### 4.2 启动容器

```bash
docker run -it --rm \
    --privileged \
    --network host \
    -v /dev:/dev \
    -v ~/ws:/ws \
    --name nexus-sdk \
    <镜像名称>
```

参数说明：

| 参数 | 说明 |
|------|------|
| `--privileged` | 允许容器访问宿主机设备（串口、输入设备等） |
| `--network host` | 共享宿主机网络，使 DDS 跨机通信直接走宿主机网卡 |
| `-v /dev:/dev` | 挂载设备目录，使容器可访问 `/dev/nexus_arm`、`/dev/input/event*` 等 |
| `-v ~/ws:/ws` | 将工作空间挂载到容器内 `/ws`，编译产物持久化到宿主机 |

### 4.3 在容器内编译

进入容器后：

```bash
cd /ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 4.4 在容器内启动遥操作

**Master 机容器内**：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py \
    role:=master \
    network_interface:=<宿主机网卡名或IP>
```

**Slave 机容器内**：

```bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py \
    role:=slave \
    network_interface:=<宿主机网卡名或IP>
```

> **注意**：容器使用 `--network host`，`network_interface` 参数填写宿主机实际网卡名，与非 Docker 环境一致。

### 4.5 多终端操作（同一容器）

如需在已运行的容器内打开新终端：

```bash
docker exec -it nexus-sdk bash
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
```

---

## 五、遥操作使用流程

### 5.1 按键说明

系统通过 `libevdev` 直接读取输入设备，**按键在任何窗口聚焦状态下均有效**（需已完成 [2.2 输入权限配置](#22-配置输入设备权限master-机必做)）。

#### 脚踏板按键（小键盘）

| 按键 | 功能 |
|------|------|
| 小键盘 `1` | 触发复位流程 |
| 小键盘 `2` | 暂停 / 恢复遥操作 |
| 小键盘 `3` | 切换计算模式（Scaling ↔ 增量） |
| 小键盘 `4` | 增大缩放系数 |
| 小键盘 `5` | 减小缩放系数 |

#### 控制按键（主键盘）

| 按键 | 功能 |
|------|------|
| `Q`（长按 3 秒） | 启动遥操作 |
| `R`（持续按住） | 安全按键，松开则暂停遥操 |
| `W` | 数据采集按键 |
| `E` | 数据标记 |
| `T` | 切换至模型推理模式 |

### 5.2 操作步骤

系统启动后按以下顺序操作：

```
1. 等待自检完成（约 5 秒）：观察日志出现 Idle 状态
2. 按 小键盘1       → 触发复位（Reset → ResetComplete）
3. 按住 R，长按 Q 3秒 → 启动遥操（TeleopRunning）
4. 移动 Nexus-Arm v05 → AR5 同步跟随
5. 松开 R           → 暂停位置保持（PositionHold）
6. 再次按 小键盘1   → 重新复位
```

### 5.3 状态流转图

```
BootSelfCheck ──自检通过──▶ Idle ──小键盘1──▶ Reset ──复位完成──▶ ResetComplete
                                                                        │
                         ┌──────────────────── Q长按3s + 按住R ─────────┘
                         ▼
                   TeleopRunning ◀──小键盘2松开── TeleopPaused
                         │    └──小键盘2按下──▶ TeleopPaused
                         └──松开R──▶ PositionHold ──小键盘1──▶ Reset
```

---

## 六、常见问题

### Q: 启动后提示找不到 `/dev/nexus_arm`

**A**: 未安装 udev 规则或 Nexus-Arm 未连接：
```bash
cd ~/ws/tool && sudo ./set_rules.sh
# 重新插拔 USB 后验证
ls -la /dev/nexus_arm
```

### Q: 按键无响应

**A**: 用户没有 `input` 组权限：
```bash
cd ~/ws/tool && sudo ./setup_pedal_permissions.sh
# 注销重新登录，或执行：
newgrp input
```

### Q: DDS 跨机发现失败，两台机器无法通信

**A**: 多网卡场景下未指定正确网卡，或两台机器的 `domain_id` 不一致：
```bash
# 查看连接到 10.18.x.x 网段的网卡
ip -4 addr | grep "10.18"

# 启动时显式指定网卡
ros2 launch nexus_manage nexus_arm_v05_to_ar5_real_system.launch.py \
    role:=master network_interface:=enp3s0
```

### Q: 编译报错找不到 OSQP 或 OsqpEigen

**A**: 安装路径不在默认位置，编译前设置环境变量：
```bash
export OSQP_INSTALL_DIR=~/osqp/install
export OSQP_EIGEN_INSTALL_DIR=~/osqp-eigen/install
colcon build --symlink-install
```

### Q: AR5 连接失败（Connecting to AR5 at 192.168.0.160 超时）

**A**: 检查 AR5 机械臂 IP 和本机 IP 配置：
```bash
# 编辑配置文件
nano ~/ws/src/teleop_adapter/config/slave_ar5_single.yaml
# 修改 right_arm_port（AR5 IP）和 right_arm_local_ip（本机 IP）
# 修改后重新编译
colcon build --symlink-install --packages-select teleop_adapter
```

### Q: Docker 容器内无法访问 `/dev/nexus_arm` 或 `/dev/input/event*`

**A**: 启动容器时必须加 `-v /dev:/dev` 和 `--privileged`，且宿主机已完成 udev 规则安装和输入权限配置。
