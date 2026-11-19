"""Deploy shallow CNN ball detector to Raspberry Pi via SSH/SCP (paramiko)."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from io import StringIO

import paramiko

RPI_HOST = "10.190.177.67"
RPI_USER = "nailong"
RPI_PASS = "nailong"
RPI_PROJECT = "/home/nailong/NUEDC_2026_H_VISION"

LOCAL_ROOT = Path(__file__).resolve().parents[3]
FILES_TO_SCP = [
    "vision_function/ball_detection/shallow_cnn/ncnn_runtime.py",
    "vision_function/debug_bridge.py",
    "vision_function/scale_location/calibration.py",
    "vision_function/scale_location/runtime.py",
    "vision_function/scale_location/deploy/vision-scale-uart.service",
    "scripts/start_rpi_gui.sh",
]


def ssh_client() -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(RPI_HOST, username=RPI_USER, password=RPI_PASS, look_for_keys=False)
    return client


def run_ssh(client: paramiko.SSHClient, cmd: str, print_output: bool = True) -> tuple[int, str]:
    """Run a command via SSH and stream output."""
    _stdin, stdout, stderr = client.exec_command(cmd)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    combined = out + ("\n[STDERR]\n" + err if err.strip() else "")
    if print_output:
        print(combined)
    return stdout.channel.recv_exit_status(), combined


def step(label: str) -> None:
    print(f"\n{'='*60}")
    print(f">>> {label}")
    print(f"{'='*60}")


def main() -> int:
    print(f"Target: {RPI_USER}@{RPI_HOST}")
    print(f"Project: {RPI_PROJECT}")

    client = ssh_client()

    # 1. Check connectivity and current state
    step("1. Connectivity & system info")
    code, out = run_ssh(client, "uname -a && ls /home/nailong/NUEDC_2026_H_VISION/vision_function/scale_location/runtime.py")
    if code != 0:
        print(f"ERROR: Cannot reach RPi project dir (exit {code})")
        return 1

    # 2. Pull latest code on RPi
    step("2. Git pull on RPi")
    run_ssh(client, f"cd {RPI_PROJECT} && git fetch origin main && git checkout main && git pull --rebase origin main")

    # 3. SCP the modified files
    step("3. SCP modified files")
    sftp = client.open_sftp()
    for rel_path in FILES_TO_SCP:
        local_path = LOCAL_ROOT / rel_path
        remote_path = f"{RPI_PROJECT}/{rel_path}"
        print(f"  {rel_path} -> {remote_path}")
        sftp.put(str(local_path), remote_path)
    sftp.close()
    print("  Done.")

    # 4. Install the single production service.  It owns both inference and
    # UART, and publishes the same computed result for the desktop monitor.
    step("4. Install single production service")
    run_ssh(client,
        f"sudo cp {RPI_PROJECT}/vision_function/scale_location/deploy/vision-scale-uart.service "
        "/etc/systemd/system/ && sudo systemctl daemon-reload && echo 'daemon-reload OK'"
    )

    # 5. Retire the temporary second service so two detector/UART processes
    # can never be enabled by the deployment path.
    step("5. Select the single shallow-CNN service")
    run_ssh(client,
        "sudo systemctl disable --now vision-scale-shallow-cnn.service 2>/dev/null || true; "
        "sudo systemctl stop vision-scale-uart.service 2>/dev/null || true; "
        "sudo systemctl enable vision-scale-uart.service && echo 'Service enabled'"
    )

    # 6. Dry-run test
    step("6. Dry-run test (5 frames)")
    code, out = run_ssh(client,
        f"cd {RPI_PROJECT} && .venv/bin/python -m vision_function.scale_location.runtime "
        "--detector shallow_cnn "
        f"--shallow-cnn-model {RPI_PROJECT}/vision_function/ball_detection/model/shallow_cnn_v3_ncnn_model "
        "--shallow-cnn-presence 0.5 --shallow-cnn-heatmap 0.25 "
        "--dry-run --iterations 5 --json-lines "
        "--status-every 1",
    )
    if code != 0:
        print(f"\n*** Dry-run FAILED (exit {code}). Check output above. ***")
        print("The service will NOT be started. Fix issues and re-run.")
        client.close()
        return 1

    # 7. Start the sole production service
    step("7. Start shallow CNN production service")
    run_ssh(client, "sudo systemctl start vision-scale-uart.service && sleep 3 && sudo systemctl status vision-scale-uart.service --no-pager -l")

    client.close()

    print(f"\n{'='*60}")
    print("Deployment complete!")
    print(f"  Check logs:  ssh {RPI_USER}@{RPI_HOST} journalctl -u vision-scale-uart.service -f")
    print(f"  Stop:        ssh {RPI_USER}@{RPI_HOST} sudo systemctl stop vision-scale-uart.service")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
