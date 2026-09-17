"""
Keep the ESP32 copy of the shared protocol implementation in sync.

Why a copy exists at all
------------------------
`shared/protocol.c` is the single source of truth for the frame format, the
CRC-32 table, and the byte-wise parser. The STM32 bootloader and the STM32
application compile it directly.

ESP-IDF cannot: whatever sits next to `main.cpp` in a component's `src/`
directory is compiled with C++ flags, and a `.c` file compiled as C++ trips
over constructs that are valid C but not C++. So the ESP32 side needs the same
code under a `.cpp` name.

Why the copy is still committed
-------------------------------
The copy used to be maintained by hand, which made it a second owner of the
same behaviour: the two files could drift silently and the resulting failures
look like a hardware or timing bug. This script removes the *hand* maintenance
— the copy is regenerated from the source of truth on every build.

It is still checked in rather than gitignored, because ESP-IDF discovers
component sources with a CMake `FILE(GLOB_RECURSE ...)` that runs before
PlatformIO's `post:` scripts. A file that does not exist at glob time is
silently absent from the build, which would fail in a confusing way on a fresh
checkout. Committing it makes a fresh clone and `idf.py` builds work, while
`--check` (wired into CI via `tools/check_protocol_mirror.py`) guarantees the
committed copy has not drifted from `shared/protocol.c`.

Usage
-----
    python3 tools/sync_protocol_mirror.py           # regenerate
    python3 tools/sync_protocol_mirror.py --check    # fail if out of date
"""

from pathlib import Path
import sys

# In a PlatformIO build this script is imported by SCons, which injects an
# `env` name. When run as a plain script from the repository root that name is
# absent. Resolve the repository root from the working directory (PlatformIO
# runs hooks with cwd = the project directory, here esp32-comm-bridge/) so the
# two invocation paths cannot disagree.
_CWD = Path.cwd()
REPOSITORY_ROOT = _CWD.parent if _CWD.name == "esp32-comm-bridge" else _CWD
RUNNING_UNDER_PLATFORMIO = "env" in globals()

SOURCE = REPOSITORY_ROOT / "shared" / "protocol.c"
TARGET = REPOSITORY_ROOT / "esp32-comm-bridge" / "src" / "protocol.cpp"


def sync(source: Path, target: Path, check_only: bool) -> int:
    if not source.is_file():
        raise RuntimeError(f"shared protocol source not found: {source}")

    text = source.read_text(encoding="utf-8")
    up_to_date = target.is_file() and target.read_text(encoding="utf-8") == text

    if up_to_date:
        return 0

    if check_only:
        print(f"[shared-protocol] {target.name} is stale; run "
              f"`python3 tools/sync_protocol_mirror.py` to regenerate it")
        return 1

    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    print(f"[shared-protocol] regenerated {target} from {source}")
    return 0


def main() -> int:
    check_only = "--check" in sys.argv[1:]
    try:
        result = sync(SOURCE, TARGET, check_only)
    except RuntimeError as exc:
        print(f"[shared-protocol] {exc}")
        return 1
    return result


# PlatformIO import path: run during the build, never exit the build process.
if RUNNING_UNDER_PLATFORMIO:
    sync(SOURCE, TARGET, check_only=False)
else:
    sys.exit(main())
