# 脚踏板权限问题解决方案

## 问题描述
普通用户运行程序时无法访问脚踏板设备，因为脚踏板通过 `/dev/input/event*` 设备文件读取，这些设备默认只有 root 用户或 input 组成员才能访问。

## 解决方案

### 方案1：使用自动配置脚本（推荐）

#### 步骤1: 运行配置脚本
```bash
sudo ./setup_pedal_permissions.sh
```

这个脚本会自动完成以下操作：
- 安装 udev 规则到 `/etc/udev/rules.d/`
- 将当前用户添加到 `input` 组
- 重新加载 udev 规则
- 立即应用设备权限

#### 步骤2: 使组成员身份生效

**选项A（推荐）**：注销并重新登录

**选项B（临时生效）**：在当前终端运行
```bash
newgrp input
```

#### 步骤3: 验证配置
```bash
./check_pedal_permissions.sh
```

如果看到 "✓ 权限配置正确，可以使用普通用户运行程序"，说明配置成功。

#### 步骤4: 启动程序
```bash
ros2 launch teleop_app dual_human_data_solver.launch.py
```

---

### 方案2：手动配置

如果自动脚本无法使用，可以手动执行以下步骤：

#### 1. 安装 udev 规则
```bash
sudo cp 99-teleop-input.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 2. 将用户添加到 input 组
```bash
sudo usermod -a -G input $USER
```

#### 3. 验证组成员身份
```bash
groups $USER
```

应该能看到 `input` 出现在列表中。

#### 4. 注销重新登录
或在当前终端运行：
```bash
newgrp input
```

#### 5. 检查设备权限
```bash
ls -l /dev/input/event*
```

应该看到类似：
```
crw-rw---- 1 root input 13, 64 ... /dev/input/event0
```

#### 6. 测试访问
```bash
cat /dev/input/event0
```

如果不报错（按 Ctrl+C 退出），说明可以访问。

---

### 方案3：临时解决（仅用于测试）

如果只是临时测试，可以直接修改设备权限（重启后失效）：

```bash
sudo chmod 660 /dev/input/event*
sudo chgrp input /dev/input/event*
```

然后在当前会话：
```bash
newgrp input
```

**注意**：此方法在系统重启后会失效，不建议用于生产环境。

---

## 验证工具

### check_pedal_permissions.sh
运行此脚本检查权限配置状态：
```bash
./check_pedal_permissions.sh
```

输出示例（配置正确）：
```
✓ 用户已在 input 组中
✓ udev 规则已安装
✓ 所有设备都可访问
✓ 可以读取设备
状态: ✓ 权限配置正确，可以使用普通用户运行程序
```

---

## 故障排除

### 问题1: "用户不在 input 组中"
```bash
sudo usermod -a -G input $USER
newgrp input  # 或注销重新登录
```

### 问题2: "无法访问任何设备"
```bash
sudo ./setup_pedal_permissions.sh
newgrp input
```

### 问题3: 重启后权限失效
检查 udev 规则是否正确安装：
```bash
ls -l /etc/udev/rules.d/99-teleop-input.rules
```

### 问题4: 仍然需要 sudo
确保：
1. 用户在 input 组中：`groups`
2. 已注销重新登录或运行 `newgrp input`
3. 设备权限正确：`ls -l /dev/input/event*`

---

## 技术细节

### udev 规则说明
文件：`99-teleop-input.rules`

```
KERNEL=="event*", SUBSYSTEM=="input", MODE="0660", GROUP="input"
```

- `KERNEL=="event*"`: 匹配所有 event 设备
- `SUBSYSTEM=="input"`: 限定为输入子系统
- `MODE="0660"`: 设置权限为 rw-rw----
- `GROUP="input"`: 设置组为 input

### 为什么需要这样做？

程序使用 `libevdev` 库读取 `/dev/input/event*` 设备来检测脚踏板按键。默认情况下，这些设备只允许 root 和 input 组成员访问。通过 udev 规则和组成员身份，我们让普通用户也能访问这些设备。

---

## 安全建议

1. **只添加需要的用户到 input 组**，因为 input 组成员可以读取所有键盘和鼠标输入
2. **使用 udev 规则**而不是直接修改权限，这样配置是持久化的
3. **定期审查** input 组成员

---

## 相关命令参考

```bash
# 查看用户所属组
groups

# 查看特定用户所属组
groups username

# 查看设备权限
ls -l /dev/input/event*

# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 测试设备访问（Ctrl+C 退出）
cat /dev/input/event0

# 查看当前会话的组
id

# 在新组环境中打开 shell
newgrp input
```
