"""Persistent unauthenticated Wi-Fi development agent for CanMV K230.

This service is intentionally limited to a trusted local network. It provides
file upload and execution on TCP 8266 and mirrors stdout/stderr on TCP 2323.
"""

import _thread
import gc
import io
import machine
import network
import os
import socket
import sys
import time


USB_PRINT = print


WIFI_SSID = "WAGA"
WIFI_PASSWORD = None
CONTROL_PORT = 8266
CONSOLE_PORT = 2323
AUTORUN_PATH = "/sdcard/app.py"
MAX_UPLOAD_BYTES = 2 * 1024 * 1024
MAX_LOG_CHUNKS = 256
WIFI_RETRY_DELAY_S = 3


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    while not wlan.isconnected():
        access_points = wlan.scan(WIFI_SSID)
        if not access_points:
            print("remote-dev: Wi-Fi not found, retrying", WIFI_SSID)
            time.sleep(WIFI_RETRY_DELAY_S)
            continue
        access_point = max(access_points, key=lambda item: item.rssi)
        print("remote-dev: selected", access_point)
        try:
            wlan.connect(None, WIFI_PASSWORD, info=access_point)
        except Exception as error:
            print("remote-dev: connect request failed", error)
            time.sleep(WIFI_RETRY_DELAY_S)
            continue
        deadline = time.ticks_add(time.ticks_ms(), 15000)
        while (not wlan.isconnected() and
               time.ticks_diff(deadline, time.ticks_ms()) > 0):
            time.sleep_ms(250)
        if not wlan.isconnected():
            print("remote-dev: Wi-Fi timeout, retrying")
            time.sleep(WIFI_RETRY_DELAY_S)
    print("remote-dev: network", wlan.ifconfig())
    return wlan


class LogHub:
    def __init__(self):
        self.lock = _thread.allocate_lock()
        self.sequence = 0
        self.chunks = []

    def publish(self, data):
        if isinstance(data, str):
            data = data.encode()
        if not data:
            return
        with self.lock:
            self.sequence += 1
            self.chunks.append((self.sequence, bytes(data)))
            if len(self.chunks) > MAX_LOG_CHUNKS:
                self.chunks.pop(0)

    def read_after(self, sequence):
        with self.lock:
            if sequence is None:
                result = self.chunks[:]
            else:
                result = [item for item in self.chunks
                          if item[0] > sequence]
            current = self.sequence
        return current, result


class RemoteDevAgent:
    def __init__(self, wlan, hub):
        self.wlan = wlan
        self.hub = hub
        self.state_lock = _thread.allocate_lock()
        self.running = False
        self.running_path = ""
        self.requested_path = ""
        self.last_result = "idle"
        self.started_ms = time.ticks_ms()

    def log_text(self, text):
        USB_PRINT(text, end="")
        self.hub.publish(text)

    def log_print(self, *args, **kwargs):
        separator = kwargs.get("sep", " ")
        ending = kwargs.get("end", "\n")
        text = separator.join([str(item) for item in args]) + ending
        self.log_text(text)

    def log_exception(self, error):
        try:
            output = io.StringIO()
            sys.print_exception(error, output)
            self.log_text(output.getvalue())
            output.close()
        except BaseException:
            sys.print_exception(error)
            self.log_print("Exception:", repr(error))

    def status(self):
        with self.state_lock:
            running = self.running
            path = self.running_path
            result = self.last_result
        return ("OK ip=%s running=%d path=%s result=%s mem_free=%d uptime_ms=%d" %
                (self.wlan.ifconfig()[0], running, path or "-", result,
                 gc.mem_free(), time.ticks_diff(time.ticks_ms(), self.started_ms)))

    def run_requested_script(self, path):
        with self.state_lock:
            self.running = True
            self.running_path = path
            self.last_result = "running"
        self.log_print("remote-dev: executing", path)
        try:
            with open(path, "r") as source:
                code = compile(source.read(), path, "exec")
            exec(code, {"__name__": "__main__", "__file__": path,
                        "print": self.log_print})
            result = "completed"
            self.log_print("remote-dev: completed", path)
        except KeyboardInterrupt:
            result = "interrupted"
            self.log_print("remote-dev: interrupted", path)
        except BaseException as error:
            result = "failed"
            self.log_exception(error)
            self.log_print("remote-dev: failed", path)
        with self.state_lock:
            self.running = False
            self.running_path = ""
            self.last_result = result
        gc.collect()

    def request_run(self, path):
        if not valid_remote_path(path):
            return "ERR invalid path"
        try:
            os.stat(path)
        except OSError:
            return "ERR file not found"
        with self.state_lock:
            if self.running or self.requested_path:
                return "ERR busy running %s" % self.running_path
            self.requested_path = path
            self.last_result = "starting"
        return "OK queued %s" % path

    def take_requested_path(self):
        with self.state_lock:
            path = self.requested_path
            self.requested_path = ""
        return path


def valid_remote_path(path):
    return (path.startswith("/sdcard/") and ".." not in path and
            path != "/sdcard/main.py" and len(path) < 192)


def recv_line(client, limit=512):
    data = bytearray()
    while len(data) < limit:
        byte = client.recv(1)
        if not byte:
            raise OSError("connection closed")
        if byte == b"\n":
            return bytes(data).decode().strip()
        if byte != b"\r":
            data.extend(byte)
    raise ValueError("command line too long")


def recv_exact(client, size):
    data = bytearray()
    while len(data) < size:
        chunk = client.recv(min(4096, size - len(data)))
        if not chunk:
            raise OSError("upload connection closed")
        data.extend(chunk)
    return data


def replace_file(path, content):
    temporary = path + ".upload"
    with open(temporary, "wb") as output:
        output.write(content)
    try:
        os.remove(path)
    except OSError:
        pass
    os.rename(temporary, path)


def handle_control(client, agent):
    client.settimeout(20)
    client.sendall(b"K230DEV 1 NOAUTH\n")
    command_line = recv_line(client)
    parts = command_line.split(" ")
    command = parts[0].upper() if parts else ""

    if command == "STATUS" and len(parts) == 1:
        client.sendall((agent.status() + "\n").encode())
        return
    if command == "RUN" and len(parts) == 2:
        client.sendall((agent.request_run(parts[1]) + "\n").encode())
        return
    if command == "PUT" and len(parts) == 3:
        path = parts[1]
        try:
            size = int(parts[2])
        except ValueError:
            client.sendall(b"ERR invalid size\n")
            return
        if not valid_remote_path(path):
            client.sendall(b"ERR invalid path\n")
            return
        if size < 0 or size > MAX_UPLOAD_BYTES:
            client.sendall(b"ERR invalid size\n")
            return
        content = recv_exact(client, size)
        replace_file(path, content)
        print("remote-dev: uploaded", path, size)
        client.sendall(("OK uploaded %s %d\n" % (path, size)).encode())
        return
    if command == "RESTART" and len(parts) == 1:
        client.sendall(b"OK restarting\n")
        time.sleep_ms(200)
        machine.reset()
        return
    client.sendall(b"ERR commands: STATUS PUT RUN RESTART\n")


def network_server(agent, hub):
    control = socket.socket()
    control.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    control.bind(("0.0.0.0", CONTROL_PORT))
    control.listen(2)
    # RT-Smart treats timeout=0 as blocking for accept(); use a positive
    # timeout so one thread can service both listeners.
    control.settimeout(0.02)

    console = socket.socket()
    console.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    console.bind(("0.0.0.0", CONSOLE_PORT))
    console.listen(1)
    console.settimeout(0.02)

    console_client = None
    console_sequence = None
    print("remote-dev: control tcp://%s:%d" %
          (agent.wlan.ifconfig()[0], CONTROL_PORT))
    print("remote-dev: console tcp://%s:%d" %
          (agent.wlan.ifconfig()[0], CONSOLE_PORT))

    while True:
        try:
            client, address = control.accept()
        except OSError:
            client = None
        if client:
            try:
                handle_control(client, agent)
            except BaseException as error:
                agent.log_exception(error)
                try:
                    client.sendall(b"ERR internal error\n")
                except OSError:
                    pass
            finally:
                client.close()

        if console_client is None:
            try:
                console_client, address = console.accept()
                console_client.settimeout(0.05)
                console_client.sendall(b"K230 remote stdout/stderr\r\n")
                console_sequence = None
                print("remote-dev: console client", address)
            except OSError:
                console_client = None

        if console_client is not None:
            try:
                console_sequence, chunks = hub.read_after(console_sequence)
                for _, payload in chunks:
                    console_client.sendall(payload)
                try:
                    payload = console_client.recv(1)
                    if payload == b"":
                        raise OSError("console disconnected")
                except OSError as error:
                    # Empty timeout errors are expected for a receive-only link.
                    if error.args and error.args[0] not in (11, 110, 116):
                        raise
            except OSError:
                console_client.close()
                console_client = None
                console_sequence = None
                print("remote-dev: console disconnected")
        time.sleep_ms(10)


def main():
    wlan = connect_wifi()
    hub = LogHub()
    agent = RemoteDevAgent(wlan, hub)
    # Route this agent's subsequent log calls through the same network hub.
    globals()["print"] = agent.log_print

    print("remote-dev: WARNING unauthenticated trusted-LAN access")
    _thread.start_new_thread(network_server, (agent, hub))

    try:
        os.stat(AUTORUN_PATH)
        print(agent.request_run(AUTORUN_PATH))
    except OSError:
        print("remote-dev: no autorun file", AUTORUN_PATH)

    while True:
        path = agent.take_requested_path()
        if path:
            agent.run_requested_script(path)
        if not wlan.isconnected():
            print("remote-dev: Wi-Fi disconnected; reconnecting")
            wlan = connect_wifi()
            agent.wlan = wlan
        time.sleep_ms(50)


if __name__ == "__main__":
    main()
