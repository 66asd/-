"""Run after Keil Rebuild. Requires Python and Unicorn 2."""
from math import pi, sin
from pathlib import Path
import random
import re
import struct

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB
from unicorn.arm_const import UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP

build = Path(__file__).resolve().parents[1] / "MDK-ARM/Sweep"
symbols = (build / "Sweep.map").read_text(errors="replace")
cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
flash, ram = 0x08000000, 0x20000000
cpu.mem_map(flash, 65536)
cpu.mem_map(ram, 20480)
cpu.mem_map(0x40000000, 0x21000)
for line in (build / "Sweep.hex").read_text().splitlines():
    record = bytes.fromhex(line[1:])
    address = int.from_bytes(record[1:3], "big")
    if record[3] == 4:
        base = int.from_bytes(record[4:6], "big") << 16
    elif record[3] == 0:
        cpu.mem_write(base + address, record[4:-1])


def symbol(name):
    return int(re.search(r"^\s*" + name + r"\s+(0x\w+)\s+(?:Thumb Code|Data)", symbols, re.M)[1], 16)


def call(name, arg0=0, arg1=0):
    stop = flash + 0xf000
    cpu.reg_write(UC_ARM_REG_R0, arg0)
    cpu.reg_write(UC_ARM_REG_R1, arg1)
    cpu.reg_write(UC_ARM_REG_LR, stop | 1)
    cpu.reg_write(UC_ARM_REG_SP, ram + 20480)
    cpu.emu_start(symbol(name), stop)
    return cpu.reg_read(UC_ARM_REG_R0)


def result(frame, fs):
    addr = ram + 0x2000
    cpu.mem_write(addr, struct.pack("<2048H", *frame))
    return call("estimate_millihz", addr, fs) / 1000


def read32(addr):
    return int.from_bytes(cpu.mem_read(addr, 4), "little")


# Drive the emitted SPI words with the peripheral ready and a wrapping millisecond clock.
spi = 0x40013000
cpu.mem_write(spi + 8, struct.pack("<I", 2))
now = 0xffffff00
call("queue_frequency", 100, 1)
for idx in range(301):
    hz = (idx % 300 + 1) * 100
    bank = (idx & 1) << 13
    tuning = ((hz << 28) + 12500000) // 25000000
    expected = ([0x2100] if idx == 0 else []) + [
        (0x4000 + 2*bank) | (tuning & 0x3fff),
        (0x4000 + 2*bank) | (tuning >> 14),
    ] + ([0xc000] if idx == 0 else []) + [0x2000 | (bank >> 2)]
    for word in expected:
        call("sweep_poll", now)
        assert read32(spi + 12) == word
    call("sweep_poll", now)
    assert read32(symbol("sweep_set_hz")) == hz
    call("sweep_poll", (now + 99) & 0xffffffff)
    now = (now + 100) & 0xffffffff
    call("sweep_poll", now)

rng = random.Random(9833)
worst = 0
for hz in range(100, 30001, 100):
    fs = 32000 if hz <= 1900 else 500000
    actual = hz * 1.0013  # independent of programmed DDS word
    phase = rng.random() * 2*pi
    data = [round(620 + 300*sin(2*pi*actual*i/fs + phase) + rng.uniform(-2, 2))
            for i in range(2048)]
    relative = abs(result(data, fs) / actual - 1)
    assert relative < 0.001, (hz, relative)
    worst = max(worst, relative)
assert result([350]*2048, 500000) == 0
print(f"300 plateaus + wrap, 300 noisy sine frames + DC passed. Worst {100*worst:.5f}%")
