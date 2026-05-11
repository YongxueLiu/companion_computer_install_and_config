# Sogou Pinyin (搜狗拼音) 安装配置指南

## 系统环境

| 项目 | 版本/信息 |
|------|----------|
| 操作系统 | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| 架构 | ARM64 (aarch64) |
| 桌面环境 | GNOME 3 |
| 内核 | Linux 5.15.148-tegra |

---

## 一、安装依赖

Sogou Pinyin 依赖于 Fcitx 输入框架，先确保已安装：

```bash
sudo apt update
sudo apt install fcitx fcitx-bin fcitx-config-gtk fcitx-ui-classic
```

### 已安装的 Fcitx 组件（本机）

```
fcitx (1:4.2.9.8-5)
fcitx-bin
fcitx-config-gtk
fcitx-data
fcitx-frontend-gtk2 / gtk3
fcitx-frontend-qt5
fcitx-module-dbus
fcitx-module-x11
fcitx-ui-classic
libfcitx-*
```

---

## 二、安装 Sogou Pinyin

### 1. 下载对应架构的安装包

**AMD64 (x86_64):**
```bash
wget https://archive.ubuntukylin.com/software/pool/partner/sogoupinyin_4.2.1.145_amd64.deb
```

**ARM64 (本机，Jetson 等 ARM 设备):**
```bash
wget https://archive.ubuntukylin.com/software/pool/partner/sogoupinyin_4.2.1.145_arm64.deb
```

### 2. 安装

```bash
sudo dpkg -i sogoupinyin_4.2.1.145_*.deb
sudo apt --fix-broken install   # 自动修复依赖
```

安装完成后，验证：
```bash
dpkg -l | grep sogoupinyin
# 应显示: sogoupinyin 4.2.1.145
```

---

## 三、配置环境变量

将以下环境变量添加到用户配置文件中，确保 GUI 应用程序能正确调用 Fcitx：

### 1. 添加到 `~/.bashrc`（终端环境）

```bash
cat >> ~/.bashrc << 'EOF'

# Fcitx input method
export GTK_IM_MODULE=fcitx
export QT_IM_MODULE=fcitx
export XMODIFIERS=@im=fcitx
EOF
```

### 2. 添加到 `~/.profile`（图形界面环境）

```bash
cat >> ~/.profile << 'EOF'

# Fcitx input method
export GTK_IM_MODULE=fcitx
export QT_IM_MODULE=fcitx
export XMODIFIERS=@im=fcitx
EOF
```

> **说明**：`GTK_IM_MODULE` 和 `QT_IM_MODULE` 让 GTK/Qt 应用调用 Fcitx，`XMODIFIERS` 是 X11 输入法的标准声明。

---

## 四、设置默认输入法框架

使用 `im-config` 将 Fcitx 设为默认输入法：

```bash
im-config -n fcitx
```

验证配置：
```bash
im-config -m
# 应显示 fcitx
```

---

## 五、配置 Fcitx 启动与触发键

### 1. 创建开机自启动

```bash
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/fcitx.desktop << 'EOF'
[Desktop Entry]
Type=Application
Exec=fcitx -r -d
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
Name[en_US]=Fcitx
Name=Fcitx
Comment=Sogou Pinyin Input Method
EOF
```

### 2. 设置触发快捷键

编辑 `~/.config/fcitx/config`，确保包含：

```ini
[Hotkey]
TriggerKey=CTRL_SPACE
```

> `CTRL_SPACE` 即 `Ctrl + 空格`，用于切换中/英文输入。

---

## 六、添加搜狗拼音到输入法列表

运行图形配置工具：

```bash
fcitx-config-gtk
```

在配置界面中：
1. 点击左下角的 **"+"** 按钮
2. **取消勾选** "Only Show Current Language"
3. 在搜索框中输入 **"Sogou"** 或 **"搜狗"**
4. 选择 **Sogou Pinyin** 并添加
5. 确保 Sogou Pinyin 在输入法列表中处于启用状态

---

## 七、关键问题修复：Pango 库冲突

### 问题现象

安装完成后，按 `Ctrl + Space` 无法调出中文输入，终端报错：

```
/opt/sogoupinyin/files/bin/sogoupinyin-service: symbol lookup error:
/lib/aarch64-linux-gnu/libpangocairo-1.0.so.0: undefined symbol: pango_font_get_hb_font
```

随后 Fcitx 崩溃（`double free or corruption`）。

### 根因分析

Sogou Pinyin 在 `/opt/sogoupinyin/files/lib/` 目录下捆绑了旧版本的 `libpango-1.0.so.0` 和 `libpangoft2-1.0.so.0`，但**没有**捆绑 `libpangocairo-1.0.so.0`。

运行时加载顺序：
- `libpangocairo-1.0.so.0` → 从**系统**加载（新版，含 `pango_font_get_hb_font`）
- `libpango-1.0.so.0` → 从 **Sogou 捆绑目录**加载（旧版，**不含**该符号）

导致符号解析失败，Sogou 服务启动即崩溃。

### 修复方法

将 Sogou 捆绑的旧版 Pango 库重命名，强制其使用系统版本：

```bash
sudo mv /opt/sogoupinyin/files/lib/libpango-1.0.so.0 \
       /opt/sogoupinyin/files/lib/libpango-1.0.so.0.bak

sudo mv /opt/sogoupinyin/files/lib/libpangoft2-1.0.so.0 \
       /opt/sogoupinyin/files/lib/libpangoft2-1.0.so.0.bak
```

### 修复后验证

```bash
# 查看 Sogou 服务依赖（应指向系统库）
ldd /opt/sogoupinyin/files/bin/sogoupinyin-service | grep pango
```

预期输出（系统库路径）：
```
libpangocairo-1.0.so.0 => /lib/aarch64-linux-gnu/libpangocairo-1.0.so.0
libpango-1.0.so.0      => /lib/aarch64-linux-gnu/libpango-1.0.so.0
libpangoft2-1.0.so.0   => /lib/aarch64-linux-gnu/libpangoft2-1.0.so.0
```

---

## 八、启动与使用

### 1. 启动 Fcitx

```bash
# 首次启动或重启
ckillall -9 fcitx sogoupinyin-watchdog sogoupinyin 2>/dev/null
fcitx -r -d
```

### 2. 验证运行状态

```bash
# 检查进程
ps aux | grep -E "fcitx|sogou" | grep -v grep
```

预期应有以下进程：
```
fcitx -r -d
fcitx-dbus-daemon
fcitx-dbus-watcher
sogoupinyin-watchdog
sogoupinyin-service   <-- 关键进程，崩溃则无法输入
```

### 3. 验证 IPC 通信

```bash
fcitx-remote
```
- 返回 `0`：英文模式
- 返回 `1`：输入法关闭
- 返回 `2`：中文输入法激活

### 4. 切换输入法

| 快捷键 | 功能 |
|--------|------|
| `Ctrl + Space` | 中/英文切换 |
| `Ctrl + Shift` | 在多个输入法之间切换 |
| `Shift` | 临时英文（搜狗内） |

---

## 九、故障排查

### 1. 仍无法输入中文

1. **必须注销并重新登录**（或重启），使环境变量生效
2. 检查当前终端/应用是否继承了环境变量：
   ```bash
   env | grep -E "GTK_IM|QT_IM|XMODIFIERS"
   ```
3. 在目标应用程序中按 `Ctrl + Space`，不是在终端里按

### 2. Fcitx 图标不显示

GNOME 3 默认隐藏传统系统托盘图标，可：
- 安装 **TopIcons Plus** 扩展
- 或使用命令切换：
  ```bash
  fcitx-remote -t
  ```

### 3. Sogou 服务反复崩溃

检查是否仍有库冲突：
```bash
ldd /opt/sogoupinyin/files/bin/sogoupinyin-service | grep "not found"
```

如有缺失库，安装对应依赖：
```bash
sudo apt --fix-broken install
sudo apt install -f
```

### 4. 词库或配置异常

重置 Sogou 配置：
```bash
mv ~/.config/sogoupinyin ~/.config/sogoupinyin.bak
mkdir -p ~/.config/sogoupinyin/log
fcitx -r -d
```

---

## 十、完整一键修复脚本

如果输入法突然失效，可运行以下脚本快速修复：

```bash
#!/bin/bash
set -e

echo "[1/4] Killing existing processes..."
killall -9 fcitx sogoupinyin-watchdog sogoupinyin-service 2>/dev/null || true
sleep 1

echo "[2/4] Fixing library conflict..."
if [ -f /opt/sogoupinyin/files/lib/libpango-1.0.so.0 ]; then
    sudo mv /opt/sogoupinyin/files/lib/libpango-1.0.so.0 \
            /opt/sogoupinyin/files/lib/libpango-1.0.so.0.bak
fi
if [ -f /opt/sogoupinyin/files/lib/libpangoft2-1.0.so.0 ]; then
    sudo mv /opt/sogoupinyin/files/lib/libpangoft2-1.0.so.0 \
            /opt/sogoupinyin/files/lib/libpangoft2-1.0.so.0.bak
fi

echo "[3/4] Ensuring config dirs..."
mkdir -p ~/.config/sogoupinyin/log
mkdir -p ~/.config/autostart

echo "[4/4] Starting fcitx..."
fcitx -r -d
sleep 2

echo "=== Status Check ==="
ps aux | grep -E "fcitx|sogou" | grep -v grep || true
fcitx-remote && echo "Fcitx IPC OK" || echo "Fcitx IPC failed"

echo "Done. Press Ctrl+Space to activate Sogou Pinyin."
```

---

## 附录：已安装软件包清单

```
sogoupinyin               4.2.1.145           (ARM64)
fcitx                     1:4.2.9.8-5
libpango-1.0-0            1.50.6+ds-2ubuntu1
libpangocairo-1.0-0       1.50.6+ds-2ubuntu1
```

---

> 文档生成时间：2026-05-09
> 适用系统：Ubuntu 22.04 LTS ARM64 + GNOME3 + Fcitx 4.x
