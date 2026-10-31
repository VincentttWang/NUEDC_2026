"""Run the uploaded K230 Wi-Fi remote development agent."""

path = "/sdcard/remote_dev_agent.py"
with open(path, "r") as source:
    code = compile(source.read(), path, "exec")
exec(code, {"__name__": "__main__"})
