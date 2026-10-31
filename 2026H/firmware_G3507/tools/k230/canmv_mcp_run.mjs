import { spawn } from "node:child_process";
import fs from "node:fs";
import net from "node:net";
import os from "node:os";
import path from "node:path";
import readline from "node:readline";


function findExtensionPath() {
    if (process.env.CANMV_EXTENSION_PATH) {
        return process.env.CANMV_EXTENSION_PATH;
    }
    const root = path.join(os.homedir(), ".vscode", "extensions");
    const candidates = fs.readdirSync(root)
        .filter((name) => name.startsWith("kendryte747.canmv-vscode-"))
        .sort()
        .reverse();
    if (candidates.length === 0) {
        throw new Error("CanMV VS Code extension is not installed");
    }
    return path.join(root, candidates[0]);
}

function parseArguments(argv) {
    const result = {
        localPath: argv[2],
        port: argv[3],
        remotePath: argv[4] || "/sdcard/mcp_script.py",
        waitMs: Number(argv[5] || 15000),
        probeRtsp: argv[6] !== "--no-probe",
    };
    if (!result.localPath) {
        throw new Error(
            "usage: node tools/k230/canmv_mcp_run.mjs <local.py> [port] [remote.py] [wait-ms]",
        );
    }
    if (!Number.isFinite(result.waitMs) || result.waitMs < 0) {
        throw new Error("wait-ms must be a non-negative number");
    }
    return result;
}

async function probeRtsp(url, timeoutMs = 5000) {
    const parsed = new URL(url);
    const port = Number(parsed.port || 554);
    return new Promise((resolve, reject) => {
        const socket = net.createConnection({ host: parsed.hostname, port });
        let response = "";
        const timer = setTimeout(() => {
            socket.destroy();
            reject(new Error(`RTSP probe timed out after ${timeoutMs} ms`));
        }, timeoutMs);

        socket.setEncoding("utf8");
        socket.on("connect", () => {
            socket.write(
                `OPTIONS ${url} RTSP/1.0\r\n` +
                "CSeq: 1\r\n" +
                "User-Agent: mspm0-car-canmv-probe\r\n\r\n",
            );
        });
        socket.on("data", (chunk) => {
            response += chunk;
            if (response.includes("\r\n\r\n")) {
                clearTimeout(timer);
                socket.end();
                resolve(response);
            }
        });
        socket.on("error", (error) => {
            clearTimeout(timer);
            reject(error);
        });
    });
}

const args = parseArguments(process.argv);
const localPath = path.resolve(args.localPath);
const script = fs.readFileSync(localPath, "utf8");
const extensionPath = findExtensionPath();
const extensionPackage = JSON.parse(fs.readFileSync(
    path.join(extensionPath, "package.json"), "utf8"));
const serverPath = path.join(extensionPath, "out", "mcp", "server.js");

const child = spawn(process.execPath, [serverPath], {
    cwd: extensionPath,
    env: {
        ...process.env,
        CANMV_EXTENSION_PATH: extensionPath,
        CANMV_EXTENSION_VERSION: extensionPackage.version,
        CANMV_BAUD_RATE: "12000000",
    },
    stdio: ["pipe", "pipe", "pipe"],
});

let requestId = 0;
const pending = new Map();
const stdout = readline.createInterface({ input: child.stdout });

stdout.on("line", (line) => {
    const message = JSON.parse(line);
    if (message.id !== undefined && pending.has(message.id)) {
        const { resolve, reject } = pending.get(message.id);
        pending.delete(message.id);
        if (message.error) {
            reject(new Error(message.error.message));
        } else {
            resolve(message.result);
        }
    }
});
child.stderr.on("data", (data) => process.stderr.write(data));

function request(method, params = {}) {
    const id = ++requestId;
    child.stdin.write(JSON.stringify({ jsonrpc: "2.0", id, method, params }) + "\n");
    return new Promise((resolve, reject) => {
        pending.set(id, { resolve, reject });
        const timer = setTimeout(() => {
            if (pending.delete(id)) {
                reject(new Error(`timeout: ${method}`));
            }
        }, 30000);
        timer.unref();
    });
}

function tool(name, toolArgs = {}) {
    return request("tools/call", { name, arguments: toolArgs });
}

function toolData(result) {
    const textPart = result?.content?.find((part) => part.type === "text")?.text;
    if (!textPart) {
        return result;
    }
    try {
        return JSON.parse(textPart);
    } catch {
        return textPart;
    }
}

try {
    await request("initialize", {
        protocolVersion: "2025-06-18",
        capabilities: {},
        clientInfo: { name: "mspm0-car-runner", version: "1.0" },
    });
    child.stdin.write(JSON.stringify({
        jsonrpc: "2.0",
        method: "notifications/initialized",
        params: {},
    }) + "\n");

    console.log("detect:", JSON.stringify(toolData(await tool("canmv_detect_boards"))));
    const connectArgs = { baudRate: 12000000 };
    if (args.port) {
        connectArgs.port = args.port;
    }
    console.log("connect:", JSON.stringify(toolData(
        await tool("canmv_connect_board", connectArgs),
    )));
    console.log("run:", JSON.stringify(toolData(await tool("canmv_write_and_run_script", {
        script,
        path: args.remotePath,
        waitMs: Math.min(args.waitMs, 10000),
        clearOutput: true,
    }))));

    if (args.waitMs > 10000) {
        await new Promise((resolve) => setTimeout(resolve, args.waitMs - 10000));
    }
    const terminal = toolData(await tool("canmv_terminal_output", { clear: false }));
    const output = terminal?.text || "";
    console.log("terminal:\n" + output);
    console.log("state:", JSON.stringify(toolData(await tool("canmv_script_running"))));

    if (args.probeRtsp) {
        const match = output.match(/rtsp:\/\/[^\s]+/i);
        if (!match) {
            throw new Error("script output did not contain an RTSP URL");
        }
        const response = await probeRtsp(match[0]);
        console.log("rtsp-probe:\n" + response.trim());
    }
} finally {
    try {
        await tool("canmv_disconnect_board");
    } catch {
        // The board may already be disconnected after a transport failure.
    }
    child.stdin.end();
    const killTimer = setTimeout(() => child.kill(), 1000);
    killTimer.unref();
}
