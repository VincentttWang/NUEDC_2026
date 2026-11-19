# 树莓派桌面视觉调试台

这是一个轻量 Tk 桌面应用，复用生产链路的 640×100 局部鱼眼校正、小球检测和色块刻度检测。界面提供原始画面、识别 ROI 叠加、球心/置信度、色块几何以及逐模块耗时。主画面和 ROI 固定显示与图传 HTML 一致的两条红色水平参考线（720p 基准 `Y=307/422`）。

## 本机或 WSL 验证

无需摄像头和树莓派即可先检查处理链路：

```bash
python -m vision_function.debug_ui.app \
  --headless vision_function/scale_location/calibration_assets/color_ruler/tube-roi-2026-07-31T10-43-52-300Z.png \
  --output tmp/debug-ui-check.png
```

若本机没有安装 `ultralytics/ncnn`，可增加 `--no-ball`，色块识别和叠加仍会运行。启动图形界面：

```bash
python -m vision_function.debug_ui.app
```

界面支持演示图片、本地图片/视频、`USB摄像头` 和本机 uStreamer 的 `http://127.0.0.1:8080/snapshot`。调试应用只读取画面，不打开 UART。

通过桌面图标启动时使用“正式服务监看模式”：小球、色块、位置、速度和 UART
状态直接来自 `vision-scale-uart.service` 发布的共享内存快照。调试台不重复运行模型，
不会停止服务或打开 UART。启动脚本使用文件锁阻止同时打开两个调试台。

## 后续部署到树莓派桌面

本阶段不要执行部署。连接树莓派后，将仓库放在 `/home/nailong/NUEDC_2026_H_VISION`，再复制桌面入口：

```bash
install -Dm644 vision_function/debug_ui/deploy/nuedc-vision-debug.desktop \
  /home/nailong/.local/share/applications/nuedc-vision-debug.desktop
cp vision_function/debug_ui/deploy/nuedc-vision-debug.desktop /home/nailong/Desktop/
chmod +x /home/nailong/Desktop/nuedc-vision-debug.desktop
```

桌面入口需要一条限定范围的 sudoers 规则，仅在正式服务意外停止时允许重新启动：

```sudoers
nailong ALL=(root) NOPASSWD: /usr/bin/systemctl start vision-scale-uart.service
```

如果仓库实际路径不同，需要同步修改 `.desktop` 文件的 `Exec` 和 `Path`。
