#!/bin/bash
# 脚踏板权限设置脚本

echo "========================================="
echo "脚踏板设备权限配置脚本"
echo "========================================="
echo ""

# 检查是否以 root 权限运行
if [ "$EUID" -ne 0 ]; then 
    echo "错误：此脚本需要 root 权限运行"
    echo "请使用: sudo ./setup_pedal_permissions.sh"
    exit 1
fi

# 获取当前用户（即使通过 sudo 运行）
REAL_USER=${SUDO_USER:-$USER}

echo "当前用户: $REAL_USER"
echo ""

# 1. 复制 udev 规则到系统目录
echo "[1/4] 安装 udev 规则..."
cp 99-teleop-input.rules /etc/udev/rules.d/
if [ $? -eq 0 ]; then
    echo "✓ udev 规则已安装到 /etc/udev/rules.d/"
else
    echo "✗ 安装 udev 规则失败"
    exit 1
fi

# 2. 将用户添加到 input 组
echo ""
echo "[2/4] 将用户添加到 'input' 组..."
usermod -a -G input $REAL_USER
if [ $? -eq 0 ]; then
    echo "✓ 用户 $REAL_USER 已添加到 input 组"
else
    echo "✗ 添加用户到 input 组失败"
    exit 1
fi

# 3. 重新加载 udev 规则
echo ""
echo "[3/4] 重新加载 udev 规则..."
udevadm control --reload-rules
udevadm trigger
if [ $? -eq 0 ]; then
    echo "✓ udev 规则已重新加载"
else
    echo "✗ 重新加载 udev 规则失败"
    exit 1
fi

# 4. 立即应用权限（无需重启）
echo ""
echo "[4/4] 应用设备权限..."
chmod 660 /dev/input/event* 2>/dev/null
chgrp input /dev/input/event* 2>/dev/null
echo "✓ 设备权限已应用"

echo ""
echo "========================================="
echo "配置完成！"
echo "========================================="
echo ""
echo "重要提示："
echo "1. 请注销并重新登录，或运行以下命令使组成员身份生效："
echo "   newgrp input"
echo ""
echo "2. 验证配置："
echo "   - 检查用户组: groups $REAL_USER"
echo "   - 检查设备权限: ls -l /dev/input/event*"
echo ""
echo "3. 之后可以使用普通用户启动程序："
echo "   ros2 launch teleop_app dual_human_data_solver.launch.py"
echo ""
