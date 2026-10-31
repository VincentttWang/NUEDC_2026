# K230 无线开发与远程调试

> **⚠ 时效说明**：K230 是 2025E 赛题时期的视觉方案。2026H 改用树莓派承担视觉与图传
> （复用原先接 K230 的那路串口），固件侧的视觉循迹中间件已随外设精简移除。本文与
> `k230/`、`tools/k230/` 下的脚本作为历史资料保留，**不描述当前在用的系统**。

本文说明 K230 在开放热点 `WAGA` 上使用的无线开发代理、电脑端命令、文件布局、协议限制和 USB 恢复
流程。固件版本、CanMV 扩展和 RTSP 媒体配置见 [`k230-development.md`](k230-development.md)。

## 目标与当前状态

无线代理用于替代日常开发中的 USB raw REPL，提供以下能力：

- 上电后自动扫描并连接 2.4 GHz 开放热点 `WAGA`；
- 通过 TCP 上传 `/sdcard` 中的 Python 文件；
- 请求执行有限程序，或上传 `app.py` 后远程复位；
- 通过独立 TCP 连接持续读取程序的 `print()` 和未捕获异常；
- 在代理保持运行的同时自动执行 `/sdcard/app.py`；
- 保留 USB raw REPL 作为代理损坏或网络不可用时的恢复通道。

当前板端和电脑端实现分别为：

- `k230/remote_dev_agent.py`：安装到 `/sdcard/main.py` 的开机代理；
- `tools/k230/k230_remote.py`：电脑端上传、执行、状态、重启和控制台工具；
- `k230/rtsp_camera_stream.py`：当前部署为 `/sdcard/app.py` 的远程摄像头程序。

已在 CanMV K230 01Studio 1G、CanMV v1.8 上验证：完整复位后自动联网，有限测试程序可经 Wi-Fi 完成
上传、重启、自运行和日志回传，随后可再次经 Wi-Fi 恢复 RTSP 应用。

## 安全边界

当前协议按用户要求**不设置密码、不加密**。控制端口允许远程上传和执行任意 Python 程序，因此只能在
可信、隔离的调试网络中使用。不要把端口映射到公网，也不要在公共 Wi-Fi、校园网或有不可信客户端的
热点上运行。

服务端仍保留以下防误操作限制，但这些限制不是安全认证：

- 上传路径必须位于 `/sdcard/`；
- 拒绝含 `..` 的路径；
- 拒绝通过网络覆盖 `/sdcard/main.py`，确保 USB 恢复入口仍存在；
- 单文件最大 2 MiB；
- 同一时刻只执行一个用户程序。

## 板端文件与启动关系

```text
/sdcard/main.py                    无线开发代理，上电自动执行
    └── /sdcard/app.py             默认用户程序，由代理自动执行

/sdcard/main_before_remote_dev.py  安装代理前的原 main.py 备份
```

代理使用一个网络管理线程同时轮询控制端口和控制台端口；用户程序在主线程运行。这样 RTSP 应用仍可
创建一个硬件编码取流线程，避免超过当前固件实测可用的工作线程数量。

## 网络端口

| 端口 | 方向 | 用途 |
|---:|---|---|
| TCP 8266 | 电脑 → K230 | 状态、上传、执行请求和系统复位 |
| TCP 2323 | K230 → 电脑 | 最近日志回放和持续 `print()` /异常输出 |
| TCP 8554 | K230 → 电脑 | 当前 `app.py` 提供的 H.264 RTSP，可按程序需要改变 |

K230 使用 DHCP。本文中的 `10.190.177.220` 是当前验证地址，不是静态配置；每次热点重启后应重新读取
USB 启动日志或热点客户端列表。

## 电脑端工作流

查询当前应用状态：

```powershell
python tools/k230/k230_remote.py --host 10.190.177.220 status
```

持续读取调试输出；连接中断后工具每 2 秒自动重连，并在重新连接时接收板端保留的最近日志：

```powershell
python tools/k230/k230_remote.py --host 10.190.177.220 console
```

只上传，不改变当前正在运行的程序：

```powershell
python tools/k230/k230_remote.py --host 10.190.177.220 \
    put local.py /sdcard/app.py
```

上传为 `/sdcard/app.py` 并执行完整系统复位：

```powershell
python tools/k230/k230_remote.py --host 10.190.177.220 deploy local.py
```

`deploy` 是无限循环程序的推荐更新方式。代理先把完整内容写入临时文件，再替换 `app.py`，随后执行
`machine.reset()`；重启后的代理会自动运行新文件。若写入后、替换过程中意外断电，代理本身仍保留在
`main.py`，可以重新上传 app.py。

若当前程序已经结束，可不复位而请求再次执行：

```powershell
python tools/k230/k230_remote.py --host 10.190.177.220 run /sdcard/app.py
```

远程系统复位：

```powershell
python tools/k230/k230_remote.py --host 10.190.177.220 restart
```

## 运行状态与输出语义

`status` 返回一行状态，例如：

```text
OK ip=10.190.177.220 running=1 path=/sdcard/app.py result=running mem_free=1852256 uptime_ms=13820
```

- `running=1`：用户程序仍在执行；
- `path`：当前程序路径；
- `result`：`idle`、`starting`、`running`、`completed`、`interrupted` 或 `failed`；
- `mem_free`：MicroPython 可用堆内存；
- `uptime_ms`：代理本次启动后的运行时间。

代理向执行环境注入网络版 `print`，所以用户脚本及其内部函数的普通 `print()` 会同时出现在 USB 和 TCP
2323。用户程序主线程抛出的未捕获异常也会回传完整 traceback。由导入模块自行替换 `print`、直接写底层
USB，或在独立原生线程中调用 `sys.print_exception()` 的输出不保证被网络控制台捕获。

控制台保存最近 256 个输出片段，并且只服务一个持续日志客户端。慢客户端不会阻塞用户程序；连接断开后
程序继续运行。

## 线协议摘要

电脑连接 TCP 8266 后，板端首先发送：

```text
K230DEV 1 NOAUTH
```

随后电脑发送一条 ASCII 命令：

```text
STATUS
RUN /sdcard/app.py
PUT /sdcard/app.py <字节数>\n<紧随其后的原始文件字节>
RESTART
```

响应以 `OK` 或 `ERR` 开头并以换行结束。该协议主要供仓库内 `k230_remote.py` 使用；修改协议时须同步
更新板端和电脑端版本。

## USB 安装与恢复

首次安装无线代理仍需 USB raw REPL：

```powershell
python tools/k230/k230_tool.py --port COM15 copy \
    /sdcard/main.py /sdcard/main_before_remote_dev.py
python tools/k230/k230_tool.py --port COM15 put \
    k230/remote_dev_agent.py /sdcard/main.py
python tools/k230/k230_tool.py --port COM15 put \
    k230/rtsp_camera_stream.py /sdcard/app.py
python tools/k230/k230_tool.py --port COM15 hard-reset
```

若无线代理无法启动，恢复原开机程序：

```powershell
python tools/k230/k230_tool.py --port COM15 copy \
    /sdcard/main_before_remote_dev.py /sdcard/main.py
python tools/k230/k230_tool.py --port COM15 hard-reset
```

CanMV v1.8 偶尔会在写文件成功后丢失 raw REPL 结束标记。安装过程中若 `put` 超时，应先用
`tools/k230/probes/k230_status_probe.py` 核对远程文件大小和开头内容，不要立即重复覆盖。

## 已知限制

- 当前无认证、无加密；
- 不支持强制终止任意正在运行的 Python 线程；无限循环程序使用 `deploy` 或 `restart` 更新；
- 代理不可通过自身协议更新 `/sdcard/main.py`；升级代理须使用 USB；
- Wi-Fi 地址由 DHCP 分配，电脑端尚未实现 mDNS 或 UDP 自动发现；
- 当前实现针对一个控制请求和一个日志客户端，不是多用户远程开发服务器。
