#!/bin/bash
# ============================================================================
# MID360 网络 systemd 自启服务安装脚本
# 功能：创建开机自启服务，直接配置 enP8p1s0 网口的静态 IP
# 用法：
#   bash install_mid360_service.sh                  # 默认 IP 192.168.1.50
#   bash install_mid360_service.sh 192.168.1.50     # 指定 IP
# ============================================================================

set -e

IFACE="enP8p1s0"
IP=${1:-192.168.1.50}
MASK=24

echo "=== MID360 网络自启服务安装 ==="
echo "网口     : $IFACE"
echo "IP       : $IP"
echo ""

# 创建 systemd 服务文件（命令直接内嵌，不依赖外部脚本）
sudo tee /etc/systemd/system/mid360-network.service > /dev/null << EOF
[Unit]
Description=MID360 Network Setup
After=network.target NetworkManager.service
Wants=network.target

[Service]
Type=oneshot
ExecStartPre=-/bin/bash -c 'nmcli dev set ${IFACE} managed no 2>/dev/null || true'
ExecStartPre=/bin/bash -c 'ip addr flush dev ${IFACE}'
ExecStartPre=/bin/bash -c 'ip link set ${IFACE} down'
ExecStartPre=/bin/bash -c 'sleep 1'
ExecStartPre=/bin/bash -c 'ip link set ${IFACE} up'
ExecStart=/bin/bash -c 'ip addr add ${IP}/${MASK} dev ${IFACE}'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable mid360-network.service

echo "[完成] 服务已安装并启用开机自启"
echo ""
echo "命令参考："
echo "  立即启动 : sudo systemctl start mid360-network.service"
echo "  查看状态 : sudo systemctl status mid360-network.service"
echo "  停止服务 : sudo systemctl stop mid360-network.service"
echo "  禁用自启 : sudo systemctl disable mid360-network.service"
echo ""

# 询问是否立即启动
read -p "是否立即启动网络配置? [Y/n] " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    sudo systemctl start mid360-network.service
    sudo systemctl status mid360-network.service --no-pager
fi
