"""Tk desktop application for MSPM0 track-parameter tuning."""

from __future__ import annotations

import argparse
import queue
import threading
from pathlib import Path

from .client import (
    PosixSerial,
    TuningClient,
    TuningClientError,
    TuningStatusError,
    load_uart_device,
)
from .parameters import PARAMETER_BY_ID, PARAMETERS
from .profiles import default_profile_directory, load_profile, save_profile
from .protocol import STATUS_BUSY
from .validation import validate_values


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_UART_CONFIG = ROOT / "vision_function" / "scale_location" / "config" / "uart.json"
HELP_DOCUMENT = Path(__file__).with_name("PARAMETER_HELP.md")


class TuningWorker(threading.Thread):
    """Own the UART and serialize all requests away from Tk's event loop."""

    def __init__(self, device: str, timeout: float, events: queue.Queue) -> None:
        super().__init__(daemon=True, name="track-tuning-uart")
        self.device = device
        self.timeout = timeout
        self.events = events
        self.commands: queue.Queue = queue.Queue()
        self.stop_event = threading.Event()

    def emit(self, kind: str, payload: object = None) -> None:
        self.events.put((kind, payload))

    def submit(self, action: str, payload: object = None) -> None:
        self.commands.put((action, payload))

    def stop(self) -> None:
        self.stop_event.set()
        self.commands.put(("stop", None))

    @staticmethod
    def _values(responses) -> dict[str, int]:
        return {
            PARAMETER_BY_ID[param_id].key: response.value
            for param_id, response in responses.items()
        }

    def _refresh(self, client: TuningClient) -> dict[str, int]:
        return self._values(client.get_all())

    def run(self) -> None:
        transport = PosixSerial(self.device)
        try:
            transport.open()
            client = TuningClient(transport, timeout=self.timeout)
            self.emit("connected", self.device)
            self.commands.put(("refresh", None))
            while not self.stop_event.is_set():
                action, payload = self.commands.get()
                if action == "stop":
                    break
                try:
                    if action == "refresh":
                        values = self._refresh(client)
                        self.emit("values", (values, "已读取 MSPM0 当前参数"))
                    elif action == "apply":
                        requested: dict[int, int] = payload
                        for index, spec in enumerate(PARAMETERS, 1):
                            if self.stop_event.is_set():
                                break
                            if spec.param_id not in requested:
                                continue
                            self.emit("progress", f"正在写入 {spec.label}…")
                            client.set(spec.param_id, requested[spec.param_id])
                        if self.stop_event.is_set():
                            break
                        values = self._refresh(client)
                        mismatches = [
                            PARAMETER_BY_ID[param_id].label
                            for param_id, expected in requested.items()
                            if values[PARAMETER_BY_ID[param_id].key] != expected
                        ]
                        if mismatches:
                            raise TuningClientError("写入后核对不一致：" + "、".join(mismatches))
                        self.emit("values", (values, f"已写入并核对 {len(requested)} 项参数"))
                    elif action == "reset":
                        client.reset_defaults()
                        values = self._refresh(client)
                        self.emit("values", (values, "已恢复固件默认值并完成回读"))
                    else:
                        raise TuningClientError(f"未知界面操作：{action}")
                except Exception as exc:
                    if isinstance(exc, TuningStatusError) and exc.response.status == STATUS_BUSY:
                        message = "MSPM0 正在执行任务，拒绝写入。请先让小车返回主菜单/闲置态。"
                    else:
                        message = f"{type(exc).__name__}: {exc}"
                    self.emit("operation_error", message)
        except Exception as exc:
            self.emit("fatal", f"无法打开 {self.device}：{type(exc).__name__}: {exc}")
        finally:
            transport.close()
            self.emit("stopped", None)


class TrackTuningApp:
    BG = "#0d0f12"
    PANEL = "#16191f"
    LINE = "#2b313b"
    FG = "#e6e9ef"
    DIM = "#929aa8"
    ACCENT = "#60a5fa"
    OK = "#4ade80"
    WARN = "#fbbf24"
    BAD = "#f87171"

    def __init__(self, root, *, device: str, timeout: float, offline: bool = False) -> None:
        import tkinter as tk
        from tkinter import ttk

        self.tk = tk
        self.ttk = ttk
        self.root = root
        self.device = device
        self.timeout = timeout
        self.offline = offline
        self.events: queue.Queue = queue.Queue()
        self.worker: TuningWorker | None = None
        self.current_values: dict[str, int] = {}
        self.entry_vars: dict[str, object] = {}
        self.current_vars: dict[str, object] = {}
        self.row_status_vars: dict[str, object] = {}
        self.busy = False
        self._updating_entries = False

        root.title("H题赛道参数调试台")
        root.configure(bg=self.BG)
        root.minsize(1080, 720)
        root.geometry("1180x790")
        root.protocol("WM_DELETE_WINDOW", self.close)

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TButton", padding=(11, 7), background="#222833", foreground=self.FG)
        style.map("TButton", background=[("active", "#303846"), ("disabled", "#171a20")])

        self.connection_text = tk.StringVar(value="离线预览" if offline else "正在连接…")
        self.footer_text = tk.StringVar(value="等待操作")
        self._build_layout()

        if offline:
            self._update_values({spec.key: spec.default for spec in PARAMETERS})
            self.footer_text.set("离线预览模式：显示固件默认值，不会打开 UART")
            self.refresh_button.configure(state="disabled")
            self.apply_button.configure(state="disabled")
            self.reset_button.configure(state="disabled")
        else:
            self._set_busy(True, "正在打开 UART 并读取全部参数…")
            self.worker = TuningWorker(device, timeout, self.events)
            self.worker.start()
        root.after(30, self._poll_events)

    def _build_layout(self) -> None:
        tk = self.tk
        header = tk.Frame(self.root, bg=self.PANEL, highlightbackground=self.LINE, highlightthickness=1)
        header.pack(fill="x")
        tk.Label(header, text="H题赛道参数调试台", fg=self.FG, bg=self.PANEL,
                 font=("Microsoft YaHei UI", 15, "bold")).pack(side="left", padx=(16, 10), pady=12)
        tk.Label(header, text=f"UART {self.device} · 115200 8N1", fg=self.DIM,
                 bg=self.PANEL).pack(side="left")
        self.connection_dot = tk.Label(header, text="●", fg=self.WARN, bg=self.PANEL,
                                       font=("Segoe UI", 12))
        self.connection_dot.pack(side="right", padx=(8, 16))
        tk.Label(header, textvariable=self.connection_text, fg=self.DIM,
                 bg=self.PANEL).pack(side="right")
        self._make_help_button(header).pack(side="right", padx=14, pady=7)

        note = tk.Frame(self.root, bg="#172238", highlightbackground="#28466f", highlightthickness=1)
        note.pack(fill="x", padx=12, pady=(12, 7))
        tk.Label(
            note,
            text="仅在小车主菜单/闲置态写入。参数保存在 MSPM0 RAM，MCU 复位后需要重新应用方案。",
            fg="#bfdbfe", bg="#172238", anchor="w",
        ).pack(fill="x", padx=12, pady=8)

        table_panel = tk.Frame(self.root, bg=self.PANEL, highlightbackground=self.LINE, highlightthickness=1)
        table_panel.pack(fill="both", expand=True, padx=12, pady=5)
        canvas = tk.Canvas(table_panel, bg=self.PANEL, highlightthickness=0)
        scrollbar = self.ttk.Scrollbar(table_panel, orient="vertical", command=canvas.yview)
        self.rows = tk.Frame(canvas, bg=self.PANEL)
        window = canvas.create_window((0, 0), window=self.rows, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        self.rows.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda e: canvas.itemconfigure(window, width=e.width))
        canvas.bind_all("<MouseWheel>", lambda e: canvas.yview_scroll(int(-e.delta / 120), "units"))
        self._build_parameter_rows()

        controls = tk.Frame(self.root, bg=self.BG)
        controls.pack(fill="x", padx=12, pady=(7, 10))
        self.refresh_button = self.ttk.Button(controls, text="读取全部", command=self.refresh)
        self.refresh_button.pack(side="left")
        self.apply_button = self.ttk.Button(controls, text="应用改动", command=self.apply)
        self.apply_button.pack(side="left", padx=7)
        self.reset_button = self.ttk.Button(controls, text="恢复固件默认值", command=self.reset_defaults)
        self.reset_button.pack(side="left")
        self.ttk.Button(controls, text="加载方案", command=self.load_profile).pack(side="right")
        self.ttk.Button(controls, text="保存方案", command=self.save_profile).pack(side="right", padx=7)

        footer = tk.Frame(self.root, bg=self.PANEL, highlightbackground=self.LINE, highlightthickness=1)
        footer.pack(fill="x")
        tk.Label(footer, textvariable=self.footer_text, fg=self.DIM, bg=self.PANEL,
                 anchor="w").pack(fill="x", padx=14, pady=8)

    def _make_help_button(self, parent):
        tk = self.tk
        canvas = tk.Canvas(parent, width=34, height=34, bg=self.PANEL,
                           highlightthickness=0, cursor="hand2")
        oval = canvas.create_oval(4, 4, 30, 30, outline=self.ACCENT, width=2)
        canvas.create_text(17, 17, text="?", fill=self.ACCENT,
                           font=("Segoe UI", 13, "bold"))
        canvas.bind("<Button-1>", lambda _e: self.show_help())
        canvas.bind("<Enter>", lambda _e: canvas.itemconfigure(oval, fill="#203354"))
        canvas.bind("<Leave>", lambda _e: canvas.itemconfigure(oval, fill=""))
        return canvas

    def _build_parameter_rows(self) -> None:
        tk = self.tk
        headings = ("参数", "MCU 当前值", "待写入值", "单位", "建议范围", "状态")
        widths = (31, 13, 13, 10, 16, 20)
        for column, (heading, width) in enumerate(zip(headings, widths)):
            tk.Label(self.rows, text=heading, width=width, fg=self.DIM, bg=self.PANEL,
                     anchor="w", font=("Microsoft YaHei UI", 9, "bold")).grid(
                         row=0, column=column, sticky="ew", padx=7, pady=7)
        self.rows.grid_columnconfigure(0, weight=3)
        self.rows.grid_columnconfigure(5, weight=2)

        group_names = {1: "分段阈值与回正", 2: "终点与停车", 3: "弯道前馈"}
        row = 1
        previous_priority = None
        for spec in PARAMETERS:
            if spec.priority != previous_priority:
                tk.Label(self.rows, text=f"优先级 {spec.priority} · {group_names[spec.priority]}",
                         fg=self.ACCENT, bg="#121820", anchor="w",
                         font=("Microsoft YaHei UI", 10, "bold")).grid(
                             row=row, column=0, columnspan=6, sticky="ew", padx=1, pady=(5, 2), ipady=5)
                row += 1
                previous_priority = spec.priority
            actual = tk.StringVar(value="--")
            desired = tk.StringVar(value="")
            status = tk.StringVar(value="未读取")
            self.current_vars[spec.key] = actual
            self.entry_vars[spec.key] = desired
            self.row_status_vars[spec.key] = status
            title = f"{spec.label}\n{spec.key} · 0x{spec.param_id:04X}"
            tk.Label(self.rows, text=title, fg=self.FG, bg=self.PANEL, anchor="w",
                     justify="left").grid(row=row, column=0, sticky="ew", padx=7, pady=5)
            tk.Label(self.rows, textvariable=actual, fg=self.OK, bg=self.PANEL,
                     anchor="w").grid(row=row, column=1, sticky="ew", padx=7)
            entry = tk.Entry(self.rows, textvariable=desired, width=13, bg="#222833", fg=self.FG,
                             insertbackground=self.FG, relief="flat", justify="right")
            entry.grid(row=row, column=2, sticky="ew", padx=7, ipady=5)
            tk.Label(self.rows, text=spec.unit, fg=self.DIM, bg=self.PANEL,
                     anchor="w").grid(row=row, column=3, sticky="ew", padx=7)
            tk.Label(self.rows, text=spec.recommended_text, fg=self.DIM, bg=self.PANEL,
                     anchor="w").grid(row=row, column=4, sticky="ew", padx=7)
            tk.Label(self.rows, textvariable=status, fg=self.DIM, bg=self.PANEL,
                     anchor="w").grid(row=row, column=5, sticky="ew", padx=7)
            desired.trace_add("write", lambda *_args, key=spec.key: self._entry_changed(key))
            row += 1

    def _entry_changed(self, key: str) -> None:
        if self._updating_entries:
            return
        text = self.entry_vars[key].get().strip()
        current = self.current_values.get(key)
        if current is None:
            self.row_status_vars[key].set("待回读")
        elif text == str(current):
            self.row_status_vars[key].set("未修改")
        else:
            self.row_status_vars[key].set("已修改")

    def _set_busy(self, busy: bool, message: str | None = None) -> None:
        self.busy = busy
        state = "disabled" if busy else "normal"
        if not self.offline:
            for button in (self.refresh_button, self.apply_button, self.reset_button):
                button.configure(state=state)
        if message:
            self.footer_text.set(message)

    def _update_values(self, values: dict[str, int]) -> None:
        self.current_values = dict(values)
        self._updating_entries = True
        try:
            for spec in PARAMETERS:
                value = values[spec.key]
                self.current_vars[spec.key].set(str(value))
                self.entry_vars[spec.key].set(str(value))
                self.row_status_vars[spec.key].set("未修改")
        finally:
            self._updating_entries = False

    def _desired_values(self) -> dict[str, int] | None:
        from tkinter import messagebox

        result: dict[str, int] = {}
        for spec in PARAMETERS:
            text = self.entry_vars[spec.key].get().strip()
            try:
                result[spec.key] = int(text, 10)
            except ValueError:
                messagebox.showerror("输入错误", f"{spec.label} 必须填写十进制整数")
                return None
        return result

    def _approve_values(self, values: dict[str, int]) -> bool:
        from tkinter import messagebox

        issues = validate_values(values)
        errors = [issue.message for issue in issues if issue.severity == "error"]
        warnings = [issue.message for issue in issues if issue.severity == "warning"]
        if errors:
            messagebox.showerror("参数约束错误", "\n\n".join(errors))
            return False
        if warnings:
            return messagebox.askyesno(
                "危险参数确认",
                "以下参数超出推荐范围或推荐关系：\n\n"
                + "\n".join("• " + warning for warning in warnings)
                + "\n\n仍要继续吗？",
                icon="warning",
            )
        return True

    def refresh(self) -> None:
        if self.worker is None or self.busy:
            return
        self._set_busy(True, "正在读取全部参数…")
        self.worker.submit("refresh")

    def apply(self) -> None:
        from tkinter import messagebox

        if self.worker is None or self.busy:
            return
        values = self._desired_values()
        if values is None or not self._approve_values(values):
            return
        if not self.current_values:
            messagebox.showerror("尚未回读", "请先成功读取 MSPM0 当前参数")
            return
        changed = {
            spec.param_id: values[spec.key]
            for spec in PARAMETERS
            if values[spec.key] != self.current_values[spec.key]
        }
        if not changed:
            messagebox.showinfo("无需写入", "所有待写入值都与 MCU 当前值一致")
            return
        if not messagebox.askyesno(
            "确认写入",
            f"将依次写入 {len(changed)} 项参数，并在完成后重新读取全部参数核对。\n\n"
            "请确认小车当前位于主菜单/闲置态。",
        ):
            return
        self._set_busy(True, f"正在写入 {len(changed)} 项参数…")
        self.worker.submit("apply", changed)

    def reset_defaults(self) -> None:
        from tkinter import messagebox

        if self.worker is None or self.busy:
            return
        if not messagebox.askyesno(
            "恢复固件默认值",
            "这会让 MSPM0 RAM 中的 13 项参数全部恢复为固件编译期默认值。\n\n"
            "请确认小车位于主菜单/闲置态。",
            icon="warning",
        ):
            return
        self._set_busy(True, "正在恢复固件默认值…")
        self.worker.submit("reset")

    def save_profile(self) -> None:
        from tkinter import filedialog, messagebox, simpledialog

        values = self._desired_values()
        if values is None or not self._approve_values(values):
            return
        name = simpledialog.askstring("方案名称", "请输入赛场方案名称：", parent=self.root)
        if name is None:
            return
        directory = default_profile_directory()
        directory.mkdir(parents=True, exist_ok=True)
        path = filedialog.asksaveasfilename(
            title="保存赛场方案", initialdir=str(directory), initialfile=(name.strip() or "track") + ".json",
            defaultextension=".json", filetypes=(("JSON 配置", "*.json"),),
        )
        if not path:
            return
        try:
            profile = save_profile(path, name, values)
        except Exception as exc:
            messagebox.showerror("保存失败", f"{type(exc).__name__}: {exc}")
            return
        self.footer_text.set(f"已保存方案“{profile.name}”：{path}")

    def load_profile(self) -> None:
        from tkinter import filedialog, messagebox

        path = filedialog.askopenfilename(
            title="加载赛场方案", initialdir=str(default_profile_directory()),
            filetypes=(("JSON 配置", "*.json"),),
        )
        if not path:
            return
        try:
            profile = load_profile(path)
        except Exception as exc:
            messagebox.showerror("加载失败", f"{type(exc).__name__}: {exc}")
            return
        self._updating_entries = True
        try:
            for spec in PARAMETERS:
                self.entry_vars[spec.key].set(str(profile.values[spec.key]))
        finally:
            self._updating_entries = False
        for spec in PARAMETERS:
            self._entry_changed(spec.key)
        self.footer_text.set(f"已加载方案“{profile.name}”；点击“应用改动”才会写入 MCU")

    def show_help(self) -> None:
        tk = self.tk
        window = tk.Toplevel(self.root)
        window.title("赛道参数作用与约束说明")
        window.configure(bg=self.BG)
        window.geometry("900x720")
        text = tk.Text(window, wrap="word", bg=self.PANEL, fg=self.FG, insertbackground=self.FG,
                       relief="flat", padx=20, pady=16, spacing1=2, spacing3=5)
        scrollbar = self.ttk.Scrollbar(window, orient="vertical", command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        text.tag_configure("h1", font=("Microsoft YaHei UI", 16, "bold"), foreground=self.ACCENT, spacing3=12)
        text.tag_configure("h2", font=("Microsoft YaHei UI", 13, "bold"), foreground=self.OK, spacing1=12, spacing3=6)
        text.tag_configure("h3", font=("Microsoft YaHei UI", 11, "bold"), foreground="#bfdbfe", spacing1=9)
        text.tag_configure("code", font=("Consolas", 10), background="#101318", foreground="#f8d477")
        try:
            lines = HELP_DOCUMENT.read_text(encoding="utf-8").splitlines()
        except Exception as exc:
            lines = ["无法读取帮助文档", "", f"{type(exc).__name__}: {exc}"]
        in_code = False
        for line in lines:
            if line.startswith("```"):
                in_code = not in_code
                continue
            if line.startswith("### "):
                text.insert("end", line[4:] + "\n", "h3")
            elif line.startswith("## "):
                text.insert("end", line[3:] + "\n", "h2")
            elif line.startswith("# "):
                text.insert("end", line[2:] + "\n", "h1")
            else:
                text.insert("end", line + "\n", "code" if in_code else None)
        text.configure(state="disabled")

    def _poll_events(self) -> None:
        from tkinter import messagebox

        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "connected":
                    self.connection_text.set(f"已连接 {payload}")
                    self.connection_dot.configure(fg=self.OK)
                elif kind == "progress":
                    self.footer_text.set(str(payload))
                elif kind == "values":
                    values, message = payload
                    self._update_values(values)
                    self._set_busy(False, message)
                elif kind == "operation_error":
                    self._set_busy(False, "操作失败；请重新读取全部参数确认 MCU 实际状态")
                    messagebox.showerror("调参失败", str(payload))
                elif kind == "fatal":
                    self.connection_text.set("连接失败")
                    self.connection_dot.configure(fg=self.BAD)
                    self._set_busy(True, str(payload))
                    messagebox.showerror("UART 连接失败", str(payload))
                elif kind == "stopped" and not self.offline:
                    self.connection_text.set("已断开")
                    self.connection_dot.configure(fg=self.BAD)
        except queue.Empty:
            pass
        try:
            self.root.after(30, self._poll_events)
        except self.tk.TclError:
            pass

    def close(self) -> None:
        if self.worker is not None:
            self.worker.stop()
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MSPM0 H2/H5/H6 赛道参数桌面调试台")
    parser.add_argument("--device", help="覆盖 uart.json 中的 UART 设备")
    parser.add_argument("--uart-config", type=Path, default=DEFAULT_UART_CONFIG)
    parser.add_argument("--timeout", type=float, default=0.8, help="单帧响应超时秒数")
    parser.add_argument("--offline", action="store_true", help="只预览界面，不打开 UART")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.timeout <= 0:
        raise SystemExit("--timeout 必须大于 0")
    device = args.device or load_uart_device(args.uart_config)
    import tkinter as tk

    root = tk.Tk()
    TrackTuningApp(root, device=device, timeout=args.timeout, offline=args.offline)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
