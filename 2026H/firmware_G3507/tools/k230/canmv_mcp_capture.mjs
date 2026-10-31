import { spawn } from "node:child_process";
import fs from "node:fs";
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

const extensionPath = findExtensionPath();
const extensionPackage = JSON.parse(fs.readFileSync(
    path.join(extensionPath, "package.json"), "utf8"));
const serverPath = path.join(extensionPath, "out", "mcp", "server.js");
const port = process.argv[2];
const outputPath = path.resolve(process.argv[3] || "canmv_preview_latest.jpg");

const child = spawn(process.execPath, [serverPath], {
    cwd: extensionPath,
    env: {
        ...process.env,
        CANMV_EXTENSION_PATH: extensionPath,
        CANMV_EXTENSION_VERSION: extensionPackage.version,
        CANMV_BAUD_RATE: "12000000",
        CANMV_MCP_OUTPUT_DIR: path.dirname(outputPath),
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
        setTimeout(() => {
            if (pending.delete(id)) {
                reject(new Error(`timeout: ${method}`));
            }
        }, 20000);
    });
}

function tool(name, args = {}) {
    return request("tools/call", { name, arguments: args });
}

try {
    await request("initialize", {
        protocolVersion: "2025-06-18",
        capabilities: {},
        clientInfo: { name: "mspm0-car-capture", version: "1.0" },
    });
    child.stdin.write(JSON.stringify({
        jsonrpc: "2.0",
        method: "notifications/initialized",
        params: {},
    }) + "\n");

    console.log(JSON.stringify(await tool("canmv_detect_boards")));
    const connectArguments = { baudRate: 12000000 };
    if (port) {
        connectArguments.port = port;
    }
    console.log(JSON.stringify(await tool("canmv_connect_board",
        connectArguments)));
    console.log(JSON.stringify(await tool("canmv_execute_file", {
        path: "/sdcard/main.py",
    })));
    await new Promise((resolve) => setTimeout(resolve, 2500));
    console.log(JSON.stringify(await tool("canmv_start_preview", {
        fps: 15,
        width: 800,
        height: 480,
    })));
    await new Promise((resolve) => setTimeout(resolve, 2500));
    console.log(JSON.stringify(await tool("canmv_script_running")));
    console.log(JSON.stringify(await tool("canmv_terminal_output", {
        clear: false,
    })));
    console.log(JSON.stringify(await tool("canmv_save_latest_frame_to_host", {
        localPath: outputPath,
        waitMs: 10000,
        fresh: true,
        overwrite: true,
    })));
    await tool("canmv_stop_preview");
    await tool("canmv_disconnect_board");
} finally {
    child.stdin.end();
    setTimeout(() => child.kill(), 1000);
}
