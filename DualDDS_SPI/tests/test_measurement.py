"""Run the built ARM measurement routine with synthetic and captured ADC codes.

Requires Python unicorn and a current Keil HEX/map build; no hardware is accessed.
"""
from pathlib import Path
import math
import re
import struct

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_SP, UC_ARM_REG_LR

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / "MDK-ARM" / "DualDDS_SPI"
MAP = (BUILD / "DualDDS_SPI.map").read_text(errors="replace")


def symbol(name):
    return int(re.search(r"^\s+" + name + r"\s+0x([0-9a-fA-F]+)\s", MAP, re.M)[1], 16)


cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x08000000, 0x20000)
cpu.mem_map(0x20000000, 0x20000)
base = 0
for line in (BUILD / "DualDDS_SPI.hex").read_text().splitlines():
    record = bytes.fromhex(line[1:])
    assert sum(record) % 256 == 0
    count, kind = record[0], record[3]
    if kind == 4:
        base = int.from_bytes(record[4:6], "big") << 16
    elif kind == 0:
        cpu.mem_write(base + int.from_bytes(record[1:3], "big"), record[4:4 + count])


def measure(codes, vdda=3000):
    s1 = [round(1800 + 200 * math.sin(2 * math.pi * 1000 * i / 100000)) for i in range(1024)]
    pairs = [a | (b << 16) for a, b in zip(s1, codes)]
    cpu.mem_write(symbol("vdda_mv"), struct.pack("<I", vdda))
    cpu.mem_write(0x20008000, struct.pack("<1024I", *pairs))
    cpu.mem_write(0x2000A000, bytes(1024))
    cpu.reg_write(UC_ARM_REG_R0, 0x20008000)
    cpu.reg_write(UC_ARM_REG_R1, 0x2000A000)
    cpu.reg_write(UC_ARM_REG_SP, 0x20007000)
    cpu.reg_write(UC_ARM_REG_LR, 0x0801FFF1)
    cpu.emu_start(symbol("measure") | 1, 0x0801FFF0, count=3000000)
    frame = bytes(cpu.mem_read(0x2000A000, 64))
    reading = struct.unpack_from("<6I", frame, 24)
    expected = int((max(codes) - min(codes)) * vdda / 4096 + 0.5)
    assert reading[2] == expected, (reading, expected)
    return reading, frame


cases = 0
max_sine_error = 0
for hz in (300, 500, 1000, 2000, 5000, 10000):
    for bias in (1000, 2000, 3000):
        for phase in (0, 0.31, 1.3, 2.7):
            codes = [round(bias + 480 / 2 * 4096 / 3000 * math.sin(2 * math.pi * hz * i / 100000 + phase)) for i in range(1024)]
            reading, frame = measure(codes)
            assert reading[0] > 0 and reading[5] == 0
            assert abs(reading[1] - 480 / (2 * math.sqrt(2))) < 3
            max_sine_error = max(max_sine_error, abs(reading[2] - 480))
            cases += 1
assert max_sine_error <= 25  # Ten samples/cycle can miss a sine peak by 4.9%.

# A harmonic-rich waveform must report acquired extrema, not sine-equivalent Vpp.
codes = [round(2000 + 300 * (math.sin(2 * math.pi * 500 * i / 100000) - 0.1 * math.sin(6 * math.pi * 500 * i / 100000))) for i in range(1024)]
reading, _ = measure(codes)
assert reading[2] > reading[1] * 2 * math.sqrt(2) * 1.05
reading, frame = measure([4095 if i % 200 < 100 else 0 for i in range(1024)])
assert reading[5] == 1 and struct.unpack_from("<I", frame, 56)[0] == 1
reading, frame = measure([2000] * 1024)
assert reading[0] == 0 and reading[2] == 0
assert struct.unpack_from("<I", frame, 56)[0] == 2

print(f"PASS: {cases} sine cases; harmonic, clipping and DC cases; exact acquired Vpp in every case.")
print(f"Largest sine peak sampling error: {max_sine_error} mV at 100 kS/s.")
