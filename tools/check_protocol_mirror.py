#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""镜像一致性检查：shared/protocol.c 与 esp32-comm-bridge/src/protocol.cpp。

背景
====
ESP-IDF 把 esp32-comm-bridge/src/ 下的源文件与 main.cpp 一起按 C++ 编译，
因此桥接固件保留了一份 protocol 实现的手工副本
（见 esp32-comm-bridge/platformio.ini 的说明）。两份实现只允许存在：

1. 注释差异：/* */ 块注释与 // 行注释；
2. 空白差异：缩进、行尾空格、空行；
3. C/C++ 固有语法差异：
   - extern "C" { ... } 链接包装（只剔除包装行，包装体内的逻辑照常比较）；
   - __cplusplus 条件编译守卫的指令行（#ifdef/#ifndef/#else/#endif，守卫体保留）；
   - static_assert / _Static_assert 断言语句（含跨行写法）。

第 3 类差异在比较前由白名单剔除，并逐行打印，不做静默放行。任何其它差异都
判定为协议实现漂移，脚本以退出码 1 结束。

用法
====
    python3 tools/check_protocol_mirror.py [协议 C 源文件] [ESP32 C++ 副本]

不传参数时使用仓库内默认路径。只依赖 Python 3 标准库。

退出码
======
    0  逻辑一致
    1  检测到漂移
    2  参数错误或文件无法读取
"""

import difflib
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

DEFAULT_C_SOURCE = REPO_ROOT / "shared" / "protocol.c"
DEFAULT_CPP_MIRROR = REPO_ROOT / "esp32-comm-bridge" / "src" / "protocol.cpp"

# 比较前允许剔除的 C/C++ 固有语法结构。
EXTERN_C_OPEN = 'extern "C" {'
GUARD_OPEN = re.compile(r"^#\s*(?:ifdef|ifndef)\s+__cplusplus\b")
GUARD_ELSE = re.compile(r"^#\s*(?:else|elif)\b")
GUARD_CLOSE = re.compile(r"^#\s*endif\b")
GUARD_NESTED_IF = re.compile(r"^#\s*(?:if|ifdef|ifndef)\b")
STATIC_ASSERT = re.compile(r"^(?:static_assert|_Static_assert)\s*\(")

WHITELIST_PREVIEW_LIMIT = 20


def strip_comments(text):
    """剥离 /* */ 与 // 注释，保留字符串/字符字面量中的内容。

    注释被替换为等量换行，因此剥离后的行号与原始文件一致。
    """
    out = []
    index = 0
    length = len(text)
    quote = None  # 当前字面量定界符（" 或 '）
    while index < length:
        char = text[index]

        if quote is not None:
            out.append(char)
            if char == "\\" and index + 1 < length:
                out.append(text[index + 1])
                index += 2
                continue
            if char == quote:
                quote = None
            index += 1
            continue

        if char in ('"', "'"):
            quote = char
            out.append(char)
            index += 1
            continue

        if text.startswith("/*", index):
            end = text.find("*/", index + 2)
            if end == -1:
                # 未闭合的块注释：其后全部视为注释
                out.append("\n" * text.count("\n", index))
                break
            out.append("\n" * text.count("\n", index, end + 2))
            index = end + 2
            continue

        if text.startswith("//", index):
            end = text.find("\n", index)
            if end == -1:
                break
            index = end
            continue

        out.append(char)
        index += 1

    return "".join(out)


def normalize_lines(text):
    """剥离注释、折叠行内空白、丢弃空行，返回 [(原始行号, 归一化内容)]。"""
    result = []
    for lineno, raw in enumerate(strip_comments(text).splitlines(), start=1):
        line = " ".join(raw.split())
        if line:
            result.append((lineno, line))
    return result


def _brace_delta(line):
    """统计一行中字符串/字符字面量之外的 { 与 } 净值。"""
    delta = 0
    index = 0
    length = len(line)
    quote = None
    while index < length:
        char = line[index]
        if quote is not None:
            if char == "\\":
                index += 2
                continue
            if char == quote:
                quote = None
            index += 1
            continue
        if char in ('"', "'"):
            quote = char
            index += 1
            continue
        if char == "{":
            delta += 1
        elif char == "}":
            delta -= 1
        index += 1
    return delta


def _extern_c_wrapper_lines(lines):
    """找出 extern "C" { ... } 包装的起始行与配对结束行下标。"""
    dropped = set()
    open_index = None
    depth = 0
    for index, (_lineno, line) in enumerate(lines):
        if open_index is None:
            if line == EXTERN_C_OPEN:
                inner = _brace_delta(line)
                if inner <= 0:
                    dropped.add(index)  # 单行内自行闭合
                else:
                    open_index = index
                    depth = inner
            continue
        depth += _brace_delta(line)
        if depth <= 0:
            dropped.add(open_index)
            dropped.add(index)
            open_index = None
            depth = 0
    return dropped


def _cpp_guard_directive_lines(lines, already_dropped):
    """找出 __cplusplus 条件编译守卫的指令行；守卫体内的代码保留比较。"""
    dropped = set()
    guard_depth = 0
    for index, (_lineno, line) in enumerate(lines):
        if index in already_dropped:
            continue
        if guard_depth > 0:
            if GUARD_CLOSE.match(line):
                guard_depth -= 1
                dropped.add(index)
            elif GUARD_NESTED_IF.match(line):
                guard_depth += 1
                dropped.add(index)
            elif GUARD_ELSE.match(line):
                dropped.add(index)
            # 守卫体不剔除，继续参与逐行比较
            continue
        if GUARD_OPEN.match(line):
            guard_depth = 1
            dropped.add(index)
    return dropped


def _static_assert_lines(lines, already_dropped):
    """找出 static_assert / _Static_assert 语句（支持跨行）。"""
    dropped = set()
    open_parens = 0
    for index, (_lineno, line) in enumerate(lines):
        if index in already_dropped:
            continue
        if open_parens > 0:
            dropped.add(index)
            open_parens += line.count("(") - line.count(")")
            continue
        if STATIC_ASSERT.match(line):
            dropped.add(index)
            open_parens = max(line.count("(") - line.count(")"), 0)
    return dropped


def apply_whitelist(lines):
    """剔除允许的 C/C++ 固有语法结构，返回 (保留行, 白名单行)。"""
    dropped = _extern_c_wrapper_lines(lines)
    dropped |= _cpp_guard_directive_lines(lines, dropped)
    dropped |= _static_assert_lines(lines, dropped)
    kept = [lines[i] for i in range(len(lines)) if i not in dropped]
    allowed = [lines[i] for i in sorted(dropped)]
    return kept, allowed


def display_path(path):
    """尽量输出相对仓库根目录的短路径。"""
    try:
        return str(path.resolve().relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def render_entry(label, path, entry):
    lineno, text = entry
    return "  [%s] %s:%d: %s" % (label, path, lineno, text)


def print_whitelist_report(label, path, allowed):
    if not allowed:
        return
    print("已知允许的差异（白名单，%s 侧共 %d 行）：" % (label.strip(), len(allowed)))
    for entry in allowed[:WHITELIST_PREVIEW_LIMIT]:
        print(render_entry(label, path, entry))
    if len(allowed) > WHITELIST_PREVIEW_LIMIT:
        print("  ... 其余 %d 行省略" % (len(allowed) - WHITELIST_PREVIEW_LIMIT))


def report_differences(c_path, cpp_path, c_kept, cpp_kept):
    matcher = difflib.SequenceMatcher(
        None,
        [text for _lineno, text in c_kept],
        [text for _lineno, text in cpp_kept],
        autojunk=False,
    )
    sections = [op for op in matcher.get_opcodes() if op[0] != "equal"]
    kind_names = {
        "replace": "两侧内容不同",
        "delete": "仅存在于 C 源文件",
        "insert": "仅存在于 C++ 副本",
    }

    print("[漂移] 忽略注释与空白后仍存在 %d 处逻辑差异：" % len(sections))
    for order, (tag, i1, i2, j1, j2) in enumerate(sections, start=1):
        print()
        print("差异 %d/%d（%s）：" % (order, len(sections), kind_names[tag]))
        for entry in c_kept[i1:i2]:
            print(render_entry("C  ", c_path, entry))
        for entry in cpp_kept[j1:j2]:
            print(render_entry("C++", cpp_path, entry))

    print()
    print("判定：两份协议实现已漂移。请同步 shared/protocol.c 与 "
          "esp32-comm-bridge/src/protocol.cpp；若差异确属允许范围，"
          "再更新本脚本的白名单。")


def main(argv):
    if len(argv) > 2:
        print("用法：python3 tools/check_protocol_mirror.py [协议 C 源文件] [ESP32 C++ 副本]")
        return 2

    c_path = Path(argv[0]) if len(argv) >= 1 else DEFAULT_C_SOURCE
    cpp_path = Path(argv[1]) if len(argv) >= 2 else DEFAULT_CPP_MIRROR

    for path in (c_path, cpp_path):
        if not path.is_file():
            print("[错误] 找不到文件：%s" % path)
            return 2

    try:
        c_text = c_path.read_text(encoding="utf-8")
        cpp_text = cpp_path.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        print("[错误] 读取文件失败：%s" % exc)
        return 2

    c_kept, c_allowed = apply_whitelist(normalize_lines(c_text))
    cpp_kept, cpp_allowed = apply_whitelist(normalize_lines(cpp_text))

    c_name = display_path(c_path)
    cpp_name = display_path(cpp_path)
    print("镜像一致性检查：%s <-> %s" % (c_name, cpp_name))
    print("剥离注释与空白后：C 侧 %d 行，C++ 侧 %d 行。" % (len(c_kept), len(cpp_kept)))
    print_whitelist_report("C  ", c_name, c_allowed)
    print_whitelist_report("C++", cpp_name, cpp_allowed)
    print()

    if len(c_kept) != len(cpp_kept) or any(
        c_text_line != cpp_text_line
        for (_c_no, c_text_line), (_cpp_no, cpp_text_line) in zip(c_kept, cpp_kept)
    ):
        report_differences(c_name, cpp_name, c_kept, cpp_kept)
        return 1

    print("[通过] 两份协议实现逻辑一致（已忽略注释、空白与 C/C++ 固有语法差异）。")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
