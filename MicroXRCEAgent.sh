cat << 'EOF' > install_microxrce_autostart.sh
#!/bin/bash

set -e

SERVICE_NAME="microxrce"
SCRIPT_PATH="/usr/local/bin/start_microxrce.sh"
DEVICE="/dev/ttyTHS1"
BAUD="921600"

echo "== 创建启动脚本 =="

sudo tee $SCRIPT_PATH > /dev/null << EOL
#!/bin/bash

# 等待串口设备出现（更稳）
while [ ! -e $DEVICE ]; do
  sleep 1
done

echo "[$(date)] Starting MicroXRCEAgent on $DEVICE..."

exec MicroXRCEAgent serial --dev $DEVICE -b $BAUD
EOL

sudo chmod +x $SCRIPT_PATH

echo "== 创建 systemd 服务 =="

sudo tee /etc/systemd/system/${SERVICE_NAME}.service > /dev/null << EOL
[Unit]
Description=Micro XRCE-DDS Agent
After=network.target

[Service]
Type=simple
ExecStart=$SCRIPT_PATH
Restart=always
RestartSec=3

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOL

echo "== 刷新 systemd =="

sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "== 设置开机自启 =="

sudo systemctl enable ${SERVICE_NAME}.service

echo "== 启动服务 =="

sudo systemctl restart ${SERVICE_NAME}.service

echo "== 当前状态 =="

sleep 2
systemctl status ${SERVICE_NAME}.service --no-pager

echo "== 完成 ✅ =="
echo "查看日志: journalctl -u ${SERVICE_NAME}.service -f"

EOF
