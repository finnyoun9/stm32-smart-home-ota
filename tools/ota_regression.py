#!/usr/bin/env python3
"""Build, validate, execute, and record the final STM32 Application OTA regression.

Typical Windows workflow:
    py -3.11 tools/ota_regression.py prepare
    py -3.11 tools/ota_regression.py bluetooth --port COM6
    py -3.11 tools/ota_regression.py web --base-url http://192.168.4.1 --verify-port COM6

``prepare`` is hardware-free. The Bluetooth and Web commands perform real
firmware updates and update the same Markdown record after each run.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import pathlib
import re
import shutil
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zlib


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_FIRMWARE = REPO_ROOT / ".pio" / "build" / "app" / "firmware.bin"
DEFAULT_RECORD = REPO_ROOT / "docs" / "ota-regression-final.md"
STATE_DIR = REPO_ROOT / "build" / "ota-regression"
STATE_FILE = STATE_DIR / "state.json"
APP_BASE = 0x08002000
APP_END = 0x0800F800
RAM_BASE = 0x20000000
RAM_END = 0x20005000
MAX_APP_SIZE = 54 * 1024


def now_text() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="seconds")


def run_capture(command: list[str], *, check: bool = True) -> subprocess.CompletedProcess[str]:
    print("+", subprocess.list2cmdline(command), flush=True)
    result = subprocess.run(
        command,
        cwd=REPO_ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    print(result.stdout, end="")
    if check and result.returncode != 0:
        raise RuntimeError(f"command failed with exit code {result.returncode}")
    return result


def git_value(*args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=REPO_ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def validate_image(path: pathlib.Path) -> dict[str, object]:
    data = path.read_bytes()
    if not data:
        raise ValueError("application image is empty")
    if len(data) > MAX_APP_SIZE:
        raise ValueError(f"application image is {len(data)} bytes; limit is {MAX_APP_SIZE}")
    if len(data) < 8:
        raise ValueError("application image is too short to contain Cortex-M vectors")

    initial_sp, reset_vector = struct.unpack_from("<II", data)
    reset_address = reset_vector & ~1
    if not RAM_BASE <= initial_sp <= RAM_END:
        raise ValueError(f"invalid initial SP 0x{initial_sp:08X}")
    if reset_vector & 1 == 0 or not APP_BASE <= reset_address < APP_END:
        raise ValueError(f"invalid Thumb reset vector 0x{reset_vector:08X}")

    return {
        "firmware": str(path.resolve()),
        "size": len(data),
        "crc32": f"0x{zlib.crc32(data) & 0xFFFFFFFF:08X}",
        "initial_sp": f"0x{initial_sp:08X}",
        "reset_vector": f"0x{reset_vector:08X}",
        "partition_used_percent": round(len(data) * 100 / MAX_APP_SIZE, 1),
    }


def extract_memory(build_output: str) -> dict[str, str]:
    memory: dict[str, str] = {}
    for name, pattern in {
        "ram": r"RAM:\s+\[[^\n]+\]\s+([0-9.]+)% \(used (\d+) bytes from (\d+) bytes\)",
        "flash": r"Flash:\s+\[[^\n]+\]\s+([0-9.]+)% \(used (\d+) bytes from (\d+) bytes\)",
    }.items():
        match = re.search(pattern, build_output)
        if match:
            memory[name] = f"{match.group(2)} / {match.group(3)} bytes ({match.group(1)}%)"
    return memory


def save_state(state: dict[str, object]) -> None:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    STATE_FILE.write_text(json.dumps(state, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def load_state() -> dict[str, object]:
    if not STATE_FILE.is_file():
        raise RuntimeError("run 'python tools/ota_regression.py prepare' first")
    return json.loads(STATE_FILE.read_text(encoding="utf-8"))


def result_cell(result: object) -> str:
    if not isinstance(result, dict):
        return "NOT RUN"
    status = result.get("status", "UNKNOWN")
    timestamp = result.get("timestamp", "")
    detail = str(result.get("detail", "")).replace("|", "\\|").replace("\n", "<br>")
    return f"{status} ({timestamp})<br>{detail}"


def display_path(value: object) -> str:
    path = pathlib.Path(str(value))
    try:
        return path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError:
        return str(path)


def render_record(state: dict[str, object], record: pathlib.Path) -> None:
    image = state["image"]
    assert isinstance(image, dict)
    versions = state["versions"]
    assert isinstance(versions, dict)
    tests = state.get("tests", {})
    assert isinstance(tests, dict)
    memory = state.get("memory", {})
    assert isinstance(memory, dict)
    manual_checks = state.get("manual_checks", {})
    assert isinstance(manual_checks, dict)
    bluetooth_pass = isinstance(tests.get("bluetooth"), dict) and tests["bluetooth"].get("status") == "PASS"
    web_pass = isinstance(tests.get("web"), dict) and tests["web"].get("status") == "PASS"
    sensors_pass = web_pass and "sensors online" in str(tests["web"].get("detail", ""))
    displays_pass = manual_checks.get("oled_tft") is True
    firmware_display = display_path(image["firmware"])
    release_display = display_path(state["release_image"])

    text = f"""# Final Application OTA Regression Record

> 自动生成/更新：`py -3.11 tools/ota_regression.py ...`
> 生成时间：{state['prepared_at']}

## 固件基线

| 项目 | 值 |
|---|---|
| Git commit | `{state['git_commit']}` |
| 工作树 | {state['git_tree']} |
| Application image | `{firmware_display}` |
| 镜像大小 | **{image['size']} bytes** / {MAX_APP_SIZE} bytes ({image['partition_used_percent']}%) |
| IEEE CRC-32 | **`{image['crc32']}`** |
| Initial SP | `{image['initial_sp']}` |
| Reset vector | `{image['reset_vector']}` |
| RAM | {memory.get('ram', '未解析')} |
| Flash | {memory.get('flash', '未解析')} |
| Protocol smoke test | {state['protocol_smoke']} |

镜像已通过与 ESP32 相同的入口校验：非空、≤54 KiB、SP 位于 STM32F103 20 KiB RAM、Thumb Reset Vector 位于 `0x08002000..0x0800F800`。

## 已核对的当前 OTA 流程

1. Bluetooth：`FW <version,size,crc>` → 带 offset ACK 的 Base64 `DATA` → `VERIFY` 回读 size/CRC → `SEND`。
2. Web：`POST /api/upload?version=N` 流式写 SPIFFS → 校验 size/CRC/SP/Reset Vector → `POST /api/start`。
3. 公共下半程：ESP32 发 `CMD_OTA_AVAILABLE` → Application 写 Flash 配置并回 `CMD_OTA_READY` → reset → Bootloader 收 `OTA_BEGIN/CHUNK/END`、擦写并校验整镜像 CRC → 标记有效并回跳 Application。
4. 最终判定：入口报告 complete 还不够；脚本会再发 `VERSION`，只有返回目标版本才记 PASS。Web 未传 `--verify-port` 时只记 PARTIAL。

## 回归结果

| 路径 | 目标版本 | 自动结果 |
|---|---:|---|
| Bluetooth SPP → ESP32 SPIFFS → STM32 | {versions['bluetooth']} | {result_cell(tests.get('bluetooth'))} |
| Web HTTP → ESP32 SPIFFS → STM32 | {versions['web']} | {result_cell(tests.get('web'))} |

## 实机验收记录

- [ ] OTA 前确认 Relay 1、Relay 2、buzzer 均处于安全状态，雾化驱动板保持断开。
- [{'x' if bluetooth_pass else ' '}] Bluetooth 回归完成后，Application 正常回跳，`VERSION` = `{versions['bluetooth']}`。
- [{'x' if web_pass else ' '}] Web 回归完成后，Application 正常回跳，`VERSION` = `{versions['web']}`。
- [{'x' if displays_pass else ' '}] OLED/TFT 正常刷新，无 HardFault、无反复复位。
- [{'x' if sensors_pass else ' '}] `/api/sensors` 返回 `online=true`，数据持续刷新。
- [ ] Relay 2 / 灯带、buzzer 的控制状态符合现场预期；不为测试主动接通未知负载。
- [ ] 若失败，保存 ESP32 USB log、页面/脚本输出、供电电压和复现步骤。

## 实机执行命令（完成后可复现）

```powershell
# 1. 蓝牙：Windows 已配对 STM32-OTA-Bridge，确认实际 outgoing COM 口后执行
py -3.11 tools/ota_regression.py bluetooth --port COM6

# 2. Web：电脑连接 ESP32 SoftAP 后执行；--verify-port 会在 OTA 后自动核对版本
py -3.11 tools/ota_regression.py web --base-url http://192.168.4.1 --verify-port COM6
```

如果 Web 回归坚持用 iPhone 页面上传，选择下面这份同一镜像，并填写版本 `{versions['web']}`：

`{release_display}`

页面显示完成后，再运行：

```powershell
py -3.11 tools/ota_regression.py verify-version --port COM6 --expected {versions['web']} --transport web
```
"""
    record.parent.mkdir(parents=True, exist_ok=True)
    record.write_text(text, encoding="utf-8")


def command_prepare(args: argparse.Namespace) -> int:
    firmware = args.firmware.resolve()
    build_output = ""
    if not args.no_build:
        build_output = run_capture(["pio", "run", "-e", "app"]).stdout
    if not firmware.is_file():
        raise FileNotFoundError(f"firmware not found: {firmware}")

    image = validate_image(firmware)
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    release_image = STATE_DIR / (
        f"application-{image['size']}B-{str(image['crc32']).removeprefix('0x')}.bin"
    )
    shutil.copy2(firmware, release_image)

    smoke_status = "SKIPPED (--no-smoke)"
    if not args.no_smoke:
        smoke_binary = STATE_DIR / ("protocol_smoke_test.exe" if os.name == "nt" else "protocol_smoke_test")
        run_capture([
            "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Ishared",
            "tools/protocol_smoke_test.c", "shared/protocol.c", "-o", str(smoke_binary),
        ])
        run_capture([str(smoke_binary)])
        smoke_status = "PASS"

    dirty = git_value("status", "--short")
    state: dict[str, object] = {
        "prepared_at": now_text(),
        "git_commit": git_value("rev-parse", "HEAD"),
        "git_tree": "clean" if not dirty else "dirty（含本次未提交改动）",
        "image": image,
        "release_image": str(release_image.resolve()),
        "memory": extract_memory(build_output),
        "protocol_smoke": smoke_status,
        "versions": {"bluetooth": args.bluetooth_version, "web": args.web_version},
        "tests": {},
        "record": str(args.record.resolve()),
    }
    save_state(state)
    render_record(state, args.record.resolve())
    print(f"Prepared: {release_image}")
    print(f"Size: {image['size']} bytes")
    print(f"CRC32: {image['crc32']}")
    print(f"Record: {args.record.resolve()}")
    return 0


def read_version(port: str, baud: int, timeout: float = 8.0) -> tuple[int | None, str]:
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pyserial is required: python -m pip install pyserial") from exc

    received = ""
    with serial.Serial(port, baud, timeout=0.2, write_timeout=5) as ser:
        ser.reset_input_buffer()
        ser.write(b"VERSION\r\n")
        ser.flush()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                received += chunk.decode("utf-8", errors="replace")
                match = re.search(r"FW Version:\s*(\d+)", received)
                if match:
                    return int(match.group(1)), received.strip()
    return None, received.strip()


def update_test(state: dict[str, object], transport: str, status: str, detail: str) -> None:
    tests = state.setdefault("tests", {})
    assert isinstance(tests, dict)
    tests[transport] = {"status": status, "timestamp": now_text(), "detail": detail}
    save_state(state)
    render_record(state, pathlib.Path(str(state["record"])))


def command_bluetooth(args: argparse.Namespace) -> int:
    state = load_state()
    versions = state["versions"]
    assert isinstance(versions, dict)
    version = args.version or int(versions["bluetooth"])
    firmware = pathlib.Path(str(state["release_image"]))
    command = [
        sys.executable, str(REPO_ROOT / "tools" / "bridge_ota.py"),
        args.port, str(firmware), "--version", str(version),
        "--baud", str(args.baud), "--timeout", str(args.timeout),
    ]
    result = run_capture(command, check=False)
    if result.returncode != 0:
        update_test(state, "bluetooth", "FAIL", f"bridge_ota exit={result.returncode}")
        return result.returncode
    time.sleep(args.reboot_wait)
    actual, response = read_version(args.port, args.baud)
    if actual != version:
        update_test(state, "bluetooth", "FAIL", f"OTA complete; VERSION expected {version}, got {actual}; {response}")
        return 1
    update_test(state, "bluetooth", "PASS", f"OTA complete; VERSION={actual}")
    print(f"PASS: Bluetooth OTA, VERSION={actual}")
    return 0


def http_json(request: urllib.request.Request, timeout: float) -> dict[str, object]:
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def command_web(args: argparse.Namespace) -> int:
    state = load_state()
    versions = state["versions"]
    image = state["image"]
    assert isinstance(versions, dict) and isinstance(image, dict)
    version = args.version or int(versions["web"])
    firmware = pathlib.Path(str(state["release_image"]))
    base_url = args.base_url.rstrip("/")

    try:
        status = http_json(urllib.request.Request(base_url + "/api/status"), args.http_timeout)
        print("Initial status:", json.dumps(status, ensure_ascii=False))
        upload_url = base_url + "/api/upload?" + urllib.parse.urlencode({"version": version})
        upload = http_json(urllib.request.Request(
            upload_url, data=firmware.read_bytes(), method="POST",
            headers={"Content-Type": "application/octet-stream"},
        ), args.http_timeout)
        print("Upload:", json.dumps(upload, ensure_ascii=False))
        expected_crc = str(image["crc32"]).removeprefix("0x")
        if int(upload.get("size", -1)) != int(image["size"]) or str(upload.get("crc32", "")).upper() != expected_crc:
            raise RuntimeError("ESP32 staged metadata does not match the local image")

        started = http_json(urllib.request.Request(
            base_url + "/api/start", data=b"", method="POST",
        ), args.http_timeout)
        print("Start:", json.dumps(started, ensure_ascii=False))

        deadline = time.monotonic() + args.timeout
        final: dict[str, object] = {}
        while time.monotonic() < deadline:
            time.sleep(args.poll_interval)
            final = http_json(urllib.request.Request(base_url + "/api/status"), args.http_timeout)
            print("Status:", json.dumps(final, ensure_ascii=False))
            if final.get("state") in ("complete", "failed"):
                break
        if final.get("state") != "complete":
            raise RuntimeError(f"Web OTA did not complete: {final}")

        detail = f"HTTP OTA complete; ESP32 size={image['size']}, CRC={image['crc32']}"
        result_status = "PARTIAL"
        if args.verify_port:
            time.sleep(args.reboot_wait)
            actual, response = read_version(args.verify_port, args.baud)
            if actual != version:
                raise RuntimeError(f"VERSION expected {version}, got {actual}; {response}")
            detail += f"; VERSION={actual}"
            result_status = "PASS"
        else:
            detail += "; VERSION not checked (add --verify-port)"

        sensors = http_json(urllib.request.Request(base_url + "/api/sensors"), args.http_timeout)
        if sensors.get("online") is not True:
            raise RuntimeError(f"OTA completed but sensor snapshot is not online: {sensors}")
        detail += (
            "; sensors online"
            f"; relay1={sensors.get('relay1')}"
            f", relay2={sensors.get('relay2')}"
            f", buzzer={sensors.get('buzzer')}"
        )
        update_test(state, "web", result_status, detail)
        print(f"{result_status}: {detail}")
        return 0
    except (OSError, ValueError, RuntimeError, urllib.error.URLError) as exc:
        update_test(state, "web", "FAIL", str(exc))
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


def command_verify_version(args: argparse.Namespace) -> int:
    state = load_state()
    actual, response = read_version(args.port, args.baud)
    if actual != args.expected:
        update_test(state, args.transport, "FAIL", f"VERSION expected {args.expected}, got {actual}; {response}")
        return 1
    prior = state.get("tests", {}).get(args.transport, {})  # type: ignore[union-attr]
    prior_detail = prior.get("detail", "manual OTA") if isinstance(prior, dict) else "manual OTA"
    update_test(state, args.transport, "PASS", f"{prior_detail}; VERSION={actual}")
    print(f"PASS: VERSION={actual}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    prepare = sub.add_parser("prepare", help="build and validate the final Application image")
    prepare.add_argument("--firmware", type=pathlib.Path, default=DEFAULT_FIRMWARE)
    prepare.add_argument("--record", type=pathlib.Path, default=DEFAULT_RECORD)
    prepare.add_argument("--bluetooth-version", type=int, default=3)
    prepare.add_argument("--web-version", type=int, default=4)
    prepare.add_argument("--no-build", action="store_true")
    prepare.add_argument("--no-smoke", action="store_true")
    prepare.set_defaults(func=command_prepare)

    bluetooth = sub.add_parser("bluetooth", help="run real Bluetooth SPP OTA and verify VERSION")
    bluetooth.add_argument("--port", required=True)
    bluetooth.add_argument("--version", type=int)
    bluetooth.add_argument("--baud", type=int, default=115200)
    bluetooth.add_argument("--timeout", type=float, default=90.0)
    bluetooth.add_argument("--reboot-wait", type=float, default=2.0)
    bluetooth.set_defaults(func=command_bluetooth)

    web = sub.add_parser("web", help="run real Web OTA and optionally verify VERSION over SPP")
    web.add_argument("--base-url", default="http://192.168.4.1")
    web.add_argument("--version", type=int)
    web.add_argument("--verify-port")
    web.add_argument("--baud", type=int, default=115200)
    web.add_argument("--timeout", type=float, default=90.0)
    web.add_argument("--http-timeout", type=float, default=15.0)
    web.add_argument("--poll-interval", type=float, default=1.0)
    web.add_argument("--reboot-wait", type=float, default=2.0)
    web.set_defaults(func=command_web)

    verify = sub.add_parser("verify-version", help="record a post-OTA VERSION check")
    verify.add_argument("--port", required=True)
    verify.add_argument("--expected", required=True, type=int)
    verify.add_argument("--transport", choices=("bluetooth", "web"), required=True)
    verify.add_argument("--baud", type=int, default=115200)
    verify.set_defaults(func=command_verify_version)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    for name in ("version", "bluetooth_version", "web_version"):
        value = getattr(args, name, None)
        if value is not None and not 1 <= value <= 0xFFFFFFFF:
            raise SystemExit(f"--{name.replace('_', '-')} must be in 1..4294967295")
    try:
        return args.func(args)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
