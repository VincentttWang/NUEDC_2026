"""树莓派连接配置，从环境变量读取，避免把地址和口令硬编码进仓库。

使用前先设置（PowerShell）：
    $env:RPI_HOST = "树莓派的IP"
    $env:RPI_USER = "登录用户名"
    $env:RPI_PASS = "登录密码"

bash：
    export RPI_HOST=树莓派的IP RPI_USER=登录用户名 RPI_PASS=登录密码

RPI_PORT 可选，默认 8080（推流服务端口）。
"""
import os
import sys


def _need(name):
    v = os.environ.get(name)
    if not v:
        sys.exit(
            f"错误：环境变量 {name} 未设置。\n"
            f"请先设置 RPI_HOST / RPI_USER / RPI_PASS，用法见 tools/rpi_config.py 顶部说明。"
        )
    return v


HOST = _need("RPI_HOST")
USER = os.environ.get("RPI_USER", "pi")
PASS = os.environ.get("RPI_PASS", "")
PORT = int(os.environ.get("RPI_PORT", "8080"))

BASE_URL = f"http://{HOST}:{PORT}"
