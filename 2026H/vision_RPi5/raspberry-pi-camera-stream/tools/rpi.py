"""在树莓派上执行命令，命令通过 stdin 传入（避免命令行转义问题）。

    echo "systemctl status ustreamer" | python tools/rpi.py

连接信息见 rpi_config.py（从环境变量读取）。
"""
import sys

import paramiko

from rpi_config import HOST, USER, PASS

cmd = sys.stdin.read()

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PASS, timeout=15,
          look_for_keys=False, allow_agent=False)

stdin, stdout, stderr = c.exec_command(cmd, timeout=180, get_pty=False)
out = stdout.read().decode("utf-8", "replace")
err = stderr.read().decode("utf-8", "replace")
rc = stdout.channel.recv_exit_status()

sys.stdout.write(out)
if err.strip():
    sys.stdout.write("\n--- STDERR ---\n" + err)
sys.stdout.write(f"\n--- EXIT {rc} ---\n")
c.close()
