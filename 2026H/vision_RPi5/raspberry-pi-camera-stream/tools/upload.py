"""上传文件到树莓派，并用 sudo 放置到目标绝对路径。

    python tools/upload.py 本地文件 /远程/绝对路径

连接信息见 rpi_config.py（从环境变量读取）。
"""
import os
import shlex
import sys

import paramiko

from rpi_config import HOST, USER, PASS

if len(sys.argv) != 3:
    sys.exit("用法: python tools/upload.py <本地文件> <远程绝对路径>")

local = sys.argv[1]
remote_final = sys.argv[2]
remote_tmp = "/tmp/" + os.path.basename(local)

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(HOST, username=USER, password=PASS, timeout=15,
          look_for_keys=False, allow_agent=False)

sftp = c.open_sftp()
sftp.put(local, remote_tmp)
sftp.close()

pw = shlex.quote(PASS)
tmp = shlex.quote(remote_tmp)
dst = shlex.quote(remote_final)
cmd = (f"echo {pw} | sudo -S cp {tmp} {dst} && "
       f"echo {pw} | sudo -S chmod 644 {dst} && ls -l {dst}")

_, out, err = c.exec_command(cmd)
print(out.read().decode("utf-8", "replace"))
e = err.read().decode("utf-8", "replace")
if e.strip():
    print("STDERR:", e)
c.close()
