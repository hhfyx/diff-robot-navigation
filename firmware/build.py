#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键构建脚本 —— 不需要 Make，也能编译出 firmware.hex

为什么需要这个脚本：
    Windows 上没有 make，而 GCC 装在中文路径（如 D:\\git\\机器人）下时，
    会因为它自己解析库路径的 bug 而链接失败。这个脚本做了两件事：
      1. 自动探测可用的 arm-none-eabi-gcc（优先 ASCII 路径）
      2. 用 subprocess 直接调编译器，绕开 make 依赖

用法：
    python build.py              编译
    python build.py --clean      先清理再编译
    python build.py --size       编译后报告体积

如果没有工具链：
    下载 https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
    解压到 D:\\armgcc\\toolchain（必须是纯英文路径！）
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build"
ELF = BUILD / "firmware.elf"
HEX = BUILD / "firmware.hex"
BIN = BUILD / "firmware.bin"

CFLAGS = [
    "-mcpu=cortex-m3", "-mthumb",
    "-Og", "-g3",
    "-std=c99", "-Wall", "-Wextra",
    "-ffunction-sections", "-fdata-sections", "-fno-common",
    "-fstack-usage",
]

LDFLAGS = [
    "-mcpu=cortex-m3", "-mthumb", "-Og", "-g3",
    "-specs=nano.specs", "-specs=nosys.specs",
    "-Wl,--gc-sections",
    "-Wl,--print-memory-usage",
]


def find_toolchain():
    """找一个能用的 arm-none-eabi-gcc，优先纯 ASCII 路径"""
    candidates = []

    # 1. PATH 里的
    for name in ("arm-none-eabi-gcc", "arm-none-eabi-gcc.exe"):
        p = shutil.which(name)
        if p:
            candidates.append(Path(p))

    # 2. 常见安装位置（ASCII 优先）
    for pattern in (
        Path("D:/armgcc/toolchain/bin"),
        Path("C:/armgcc/toolchain/bin"),
        Path(os.environ.get("USERPROFILE", "")) / ".platformio/packages/toolchain-gccarmnoneeabi/bin",
        Path("C:/Program Files (x86)/GNU Arm Embedded Toolchain"),
    ):
        if pattern.is_dir():
            for exe in pattern.glob("arm-none-eabi-gcc*"):
                candidates.append(exe)

    # 3. 本项目目录下的 .toolchain（可能是中文路径，最后才用）
    local = HERE.parent / ".toolchain"
    if local.is_dir():
        for exe in local.rglob("arm-none-eabi-gcc.exe"):
            candidates.append(exe)

    # 按"路径是否纯 ASCII"排序：非 ASCII 路径下 GCC 会找不到自己的库
    def ascii_penalty(p):
        try:
            str(p).encode("ascii")
            return 0
        except UnicodeEncodeError:
            return 1

    candidates.sort(key=ascii_penalty)

    for c in candidates:
        if c.exists():
            try:
                r = subprocess.run([str(c), "--version"],
                                   capture_output=True, text=True, timeout=10)
                if r.returncode == 0:
                    return c
            except Exception:
                continue

    return None


def run(cmd, **kw):
    """执行命令，失败时打印并退出"""
    r = subprocess.run([str(x) for x in cmd], capture_output=True, text=True, **kw)
    if r.returncode != 0:
        print(f"\n[失败] {' '.join(str(x) for x in cmd[:3])} ...")
        if r.stdout:
            print(r.stdout)
        if r.stderr:
            print(r.stderr)
        sys.exit(1)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean", action="store_true")
    ap.add_argument("--size", action="store_true")
    ap.add_argument("--release", action="store_true", help="用 -O2 优化（体积更小，但调试困难）")
    args = ap.parse_args()

    gcc = find_toolchain()
    if gcc is None:
        print("找不到 arm-none-eabi-gcc。")
        print("请从 https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads")
        print("下载并解压到  D:\\armgcc\\toolchain  （必须是纯英文路径）")
        sys.exit(1)

    bindir = gcc.parent
    suffix = ".exe" if gcc.suffix.lower() == ".exe" else ""
    objcopy = bindir / ("arm-none-eabi-objcopy" + suffix)
    size_tool = bindir / ("arm-none-eabi-size" + suffix)

    print(f"工具链: {gcc}")
    ver = subprocess.run([str(gcc), "--version"], capture_output=True, text=True)
    print(f"版本  : {ver.stdout.splitlines()[0]}")

    if args.clean and BUILD.exists():
        shutil.rmtree(BUILD)
        print("已清理 build/")

    BUILD.mkdir(parents=True, exist_ok=True)

    cflags = list(CFLAGS)
    if args.release:
        cflags = [f for f in cflags if f != "-Og"] + ["-O2"]

    # --- 编译 C ---
    objs = []
    srcs = sorted((HERE / "Src").glob("*.c"))
    print(f"\n编译 {len(srcs)} 个源文件...")
    for src in srcs:
        obj = BUILD / (src.stem + ".o")
        run([gcc, *cflags, f"-I{HERE / 'Inc'}", "-c", src, "-o", obj])
        print(f"  OK  {src.name}")
        objs.append(obj)

    # --- 汇编启动文件 ---
    startup = HERE / "Startup" / "startup_stm32f10x_md.s"
    start_obj = BUILD / "startup.o"
    run([gcc, "-mcpu=cortex-m3", "-mthumb", "-c", startup, "-o", start_obj])
    print("  OK  startup_stm32f10x_md.s")
    objs.append(start_obj)

    # --- 链接 ---
    print("\n链接...")
    run([gcc, *LDFLAGS,
         f"-T{HERE / 'STM32F103C8Tx_FLASH.ld'}",
         f"-Wl,-Map={BUILD / 'firmware.map'},--cref",
         "-o", ELF, *objs])
    print(f"  OK  {ELF.name}")

    # --- 转 hex / bin ---
    run([objcopy, "-O", "ihex", ELF, HEX])
    run([objcopy, "-O", "binary", "-S", ELF, BIN])
    print(f"  OK  {HEX.name} ({HEX.stat().st_size} 字节)")
    print(f"  OK  {BIN.name} ({BIN.stat().st_size} 字节)")

    if args.size or True:
        print()
        r = subprocess.run([str(size_tool), str(ELF)],
                           capture_output=True, text=True)
        print(r.stdout)

        # 顺便检查栈占用，超过 512 字节就提醒
        su_files = sorted(BUILD.glob("*.su"))
        max_stack = 0
        worst = ""
        for su in su_files:
            for line in su.read_text(errors="ignore").splitlines():
                parts = line.split("\t")
                if len(parts) >= 2:
                    try:
                        v = int(parts[1])
                        if v > max_stack:
                            max_stack, worst = v, parts[0]
                    except ValueError:
                        pass
        if max_stack:
            print(f"单函数最大栈占用: {max_stack} 字节 ({worst})")
            if max_stack > 512:
                print("  [警告] 超过 512 字节，考虑加大 ld 里的栈大小")

    print("\n构建完成。烧录：")
    print(f"  st-flash --reset write {BIN} 0x8000000")
    print("或直接用 ST-Link Utility 打开 firmware.hex")


if __name__ == "__main__":
    main()
