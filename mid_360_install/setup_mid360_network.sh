#!/bin/bash
# ============================================================================
# MID360 机载网络配置脚本
# 功能：配置静态IP，确保电脑和 MID360 雷达在同一网段
# 用法：
#   bash setup_mid360_network.sh                  # 自动检测网口，默认IP
#   bash setup_mid360_network.sh eth0             # 指定网口，默认IP
#   bash setup_mid360_network.sh eth0 192.168.1.50 # 指定网口和IP
# ============================================================================

IFACE=${1:-""}
HOST_IP=${2:-192.168.1.50}
MASK=24

# 自动检测有线网口（排除 lo 和无线网口 wl*）
if [ -z "$IFACE" ]; then
    IFACE=$(ip -o link show | awk -F': ' '$2 !~ /^(lo|wl)/ {print $2; exit}')
    if [ -z "$IFACE" ]; then
        echo "[错误] 未检测到可用的有线网口"
        echo "系统所有网口："
        ip -o link show | awk -F': ' '{print "  " $2}'
        echo ""
        echo "请手动指定网口名，例如："
        echo "  bash setup_mid360_network.sh eth0"
        exit 1
    fi
    echo "[信息] 自动检测到网口: $IFACE"
fi

# 检查网口是否存在
if ! ip link show "$IFACE" >/dev/null 2>&1; then
    echo "[错误] 网口 '$IFACE' 不存在"
    echo "系统所有网口："
    ip -o link show | awk -F': ' '{print "  " $2}'
    exit 1
fi

echo "=== MID360 网络配置 ==="
echo "网口    : $IFACE"
echo "电脑IP  : $HOST_IP/$MASK"
echo ""

# 1. 关闭 NetworkManager 对该网口的管理（防止自动改IP）
echo "[1/4] 关闭 NetworkManager 管理..."
sudo nmcli dev set "$IFACE" managed no 2>/dev/null || true

# 2. 清空并重启网口
echo "[2/4] 清空并重启网口..."
sudo ip addr flush dev "$IFACE"
sudo ip link set "$IFACE" down
sleep 1
sudo ip link set "$IFACE" up

# 3. 设置静态IP
echo "[3/4] 设置静态IP..."
sudo ip addr add "${HOST_IP}/${MASK}" dev "$IFACE"

# 4. 验证
echo "[4/4] 验证配置..."
echo ""
ip addr show "$IFACE" | grep "inet "
echo ""
echo "[完成] 网络已配置。"
echo "  电脑IP : $HOST_IP"
echo "  子网   : $(echo $HOST_IP | cut -d. -f1-3).0/24"
echo "  雷达IP : 需和电脑在同一网段（如 192.168.1.135）"
echo ""
echo "如需恢复 NetworkManager 管理："
echo "  sudo nmcli dev set $IFACE managed yes"
