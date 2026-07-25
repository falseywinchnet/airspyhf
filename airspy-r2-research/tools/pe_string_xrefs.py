#!/usr/bin/env python3
"""Locate RIP-relative references to selected strings in a PE32+ executable."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from pathlib import Path

import pefile
from capstone import Cs, CS_ARCH_X86, CS_MODE_64
from capstone.x86 import X86_OP_IMM, X86_OP_MEM, X86_REG_RIP


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("exe", type=Path)
    parser.add_argument("patterns", nargs="+")
    parser.add_argument("--window", type=int, default=12)
    args = parser.parse_args()

    raw = args.exe.read_bytes()
    pe = pefile.PE(data=raw, fast_load=False)
    base = pe.OPTIONAL_HEADER.ImageBase
    image = pe.get_memory_mapped_image()
    text = next(s for s in pe.sections if s.Name.rstrip(b"\0") == b".text")
    code_rva = text.VirtualAddress
    code = image[code_rva : code_rva + text.Misc_VirtualSize]

    functions: list[tuple[int, int]] = []
    for entry in getattr(pe, "DIRECTORY_ENTRY_EXCEPTION", []):
        functions.append((entry.struct.BeginAddress, entry.struct.EndAddress))
    functions.sort()
    starts = [start for start, _ in functions]

    md = Cs(CS_ARCH_X86, CS_MODE_64)
    md.detail = True
    instructions = list(md.disasm(code, base + code_rva))
    by_address = {ins.address: n for n, ins in enumerate(instructions)}

    targets: dict[int, str] = {}
    lower_image = image.lower()
    for pattern in args.patterns:
        needle = pattern.encode().lower()
        cursor = 0
        while True:
            found = lower_image.find(needle, cursor)
            if found < 0:
                break
            end = image.find(b"\0", found)
            if end < 0:
                end = min(found + 160, len(image))
            targets[base + found] = image[found:end].decode("utf-8", "replace")
            cursor = found + 1

    for ins in instructions:
        referenced: list[int] = []
        for operand in ins.operands:
            if operand.type == X86_OP_MEM and operand.mem.base == X86_REG_RIP:
                referenced.append(ins.address + ins.size + operand.mem.disp)
            elif operand.type == X86_OP_IMM:
                referenced.append(operand.imm)
        for target in referenced:
            if target not in targets:
                continue
            rva = ins.address - base
            pos = bisect_right(starts, rva) - 1
            fn = functions[pos] if pos >= 0 and functions[pos][0] <= rva < functions[pos][1] else (rva, rva)
            print(
                f"string={targets[target]!r}\n"
                f"xref=0x{ins.address:x} function=0x{base + fn[0]:x}..0x{base + fn[1]:x}"
            )
            center = by_address[ins.address]
            lo = max(0, center - args.window)
            hi = min(len(instructions), center + args.window + 1)
            for n in range(lo, hi):
                item = instructions[n]
                marker = ">" if n == center else " "
                print(f"{marker} 0x{item.address:x}: {item.mnemonic:8s} {item.op_str}")
            print()


if __name__ == "__main__":
    main()

