# 树莓派赛道参数调试台

该 Tk 桌面应用通过 `/dev/ttyAMA0` 与 MSPM0 通信，读取和修改 H2/H5/H6 的 13 项赛道参数。协议帧类型为请求 `0x81`、响应 `0x82`，与现有视觉帧 `0x01` 共用物理 UART。

应用支持：

- 启动后 GET_ALL 回读；
- 单项/批量 SET 后再次 GET_ALL 核对；
- BUSY、超时、CRC 和参数集合错误提示；
- 推荐范围、硬约束和参数关系检查；
- 带版本号的赛场 profile 保存与加载；
- 右上角圆形 `?` 按钮打开内置参数说明。

参数详细说明见 [PARAMETER_HELP.md](PARAMETER_HELP.md)。

## 本机离线预览

离线模式不会打开 UART，也不会操作 systemd：

```bash
python -m vision_function.track_tuning.app --offline
```

## 树莓派启动

应从桌面图标或启动脚本进入，使正式视觉服务在调参期间暂停：

```bash
/bin/bash scripts/start_track_tuning.sh
```

不要在 `vision-scale-uart.service` 仍运行时直接打开调参模块，否则两个进程会争用 `/dev/ttyAMA0`。

## 桌面入口部署

本阶段代码只保留在本机。需要部署到树莓派时，在仓库根目录执行：

```bash
install -Dm644 vision_function/track_tuning/deploy/nuedc-track-tuning.desktop \
  /home/nailong/.local/share/applications/nuedc-track-tuning.desktop
cp vision_function/track_tuning/deploy/nuedc-track-tuning.desktop /home/nailong/Desktop/
chmod +x /home/nailong/Desktop/nuedc-track-tuning.desktop
sudo install -Dm440 vision_function/track_tuning/deploy/90-nuedc-uart-takeover \
  /etc/sudoers.d/90-nuedc-uart-takeover
sudo visudo -cf /etc/sudoers.d/90-nuedc-uart-takeover
```

若仓库路径或用户名不是 `/home/nailong/NUEDC_2026_H_VISION` 和 `nailong`，应先同步修改 desktop 文件和 sudoers 文件。上述命令当前没有执行。

## 硬件连接

调参需要双向 UART：

| Raspberry Pi 5 | MSPM0 |
|---|---|
| GPIO14 / TXD0 / Pin 8 | PA24 / UART2 RX |
| GPIO15 / RXD0 / Pin 10 | PB15 / UART2 TX |
| GND | GND |

双方均使用 3.3 V TTL、115200 8N1。当前仓库配置明确使用 `/dev/ttyAMA0`，不要改用 Pi 5 Debug Header 对应的 `/dev/serial0`。

## Profile 位置

默认目录为：

```text
~/.config/nuedc-track-tuning/profiles/
```

加载 profile 只会填充“待写入值”。必须再点击“应用改动”，应用才会逐项 SET，并以最终 GET_ALL 回读结果确认是否成功。
