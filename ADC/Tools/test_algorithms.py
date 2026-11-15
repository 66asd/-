"""Compile the actual C modules for Cortex-M3 and execute them with Unicorn."""
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time
import os

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build'
BUILD.mkdir(exist_ok=True)
acquisition_test = '--acquisition' in sys.argv
vofa_test = '--vofa' in sys.argv
assert not (acquisition_test and vofa_test)
points = int(sys.argv[sys.argv.index('--points')+1]) if '--points' in sys.argv else 16
if points not in (16,64):
    raise SystemExit('--points must be 16 or 64')
sources = ['scope_acquisition.c'] if acquisition_test else ['measure_frequency.c', 'measure_rms.c', 'measure_vpp.c',
           'sine_fit.c', 'scope_waveform.c', 'hmi_protocol.c', 'scope_app.c', 'sample_rate.c', 'vofa_raw.c']
if vofa_test:
    sources = ['vofa_raw.c', 'hmi_uart.c', 'hmi_protocol.c']
test_source = ROOT / ('Tests/test_acquisition.c' if acquisition_test else 'Tests/test_algorithms.c')
fake_includes = ROOT / ('Tests/acquisition_fakes' if acquisition_test else 'Tests/fakes')
elf = BUILD / ('acquisition-tests.elf' if acquisition_test else 'algorithm-tests.elf')
if vofa_test:
    test_source = ROOT / 'Tests/test_vofa.c'
    fake_includes = ROOT / 'Tests/vofa_fakes'
    elf = BUILD / 'vofa-tests.elf'
keil = Path(os.environ.get('KEIL_ROOT', 'D:/keil5')) / 'ARM/ARMCC/bin'
if (keil / 'armcc.exe').exists():
    objects = []
    for source in [test_source] + [ROOT / 'App' / name for name in sources]:
        obj = BUILD / ('test-' + source.stem + '.o')
        subprocess.run([str(keil / 'armcc.exe'), '--cpu=Cortex-M3', '--c99', '-O2', '-g', '-DSCOPE_TARGET_POINTS='+str(points),
                        '--split_sections', '--diag_error=warning', '-I' + str(fake_includes),
                        '-I' + str(ROOT / 'App'),
                        '-c', str(source), '-o', str(obj)], check=True)
        objects.append(str(obj))
    startup = BUILD / 'test-startup.o'
    subprocess.run([str(keil / 'armasm.exe'), '--cpu=Cortex-M3',
                    str(ROOT / 'MDK-ARM/startup_stm32f103xb.s'), '-o', str(startup)], check=True)
    subprocess.run([str(keil / 'armlink.exe'), '--cpu=Cortex-M3', '--entry=Reset_Handler',
                    '--ro_base=0x08000000', '--rw_base=0x20000000', '--first=__Vectors',
                    str(startup)] + objects + ['-o', str(elf)], check=True)
else:
    gcc = shutil.which('arm-none-eabi-gcc')
    if not gcc:
        raise SystemExit('Keil ARMCC or arm-none-eabi-gcc is required')
    cmd = [gcc, '-mcpu=cortex-m3', '-mthumb', '-O2', '-g', '-std=c99',
           '-Wall', '-Wextra', '-Werror', '-DSCOPE_TARGET_POINTS='+str(points), '-ffunction-sections', '-fdata-sections',
           '-I' + str(fake_includes), '-I' + str(ROOT / 'App'), '-nostartfiles', '--specs=nosys.specs',
           '-Wl,--gc-sections', '-T' + str(ROOT / 'Tests/test_memory.ld'),
           str(test_source)]
    cmd += [str(ROOT / 'App' / name) for name in sources]
    cmd += ['-lm', '-o', str(elf)]
    subprocess.run(cmd, check=True)

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_INTR
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC

data = elf.read_bytes()
if data[:7] != b'\x7fELF\x01\x01\x01':
    raise SystemExit('Expected a little-endian ELF32 file')
header = struct.unpack_from('<HHIIIIIHHHHHH', data, 16)
entry, phoff, phsize, phcount = header[3], header[4], header[8], header[9]
cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
cpu.mem_map(0x08000000, 512 * 1024)
cpu.mem_map(0x20000000, 128 * 1024)
for i in range(phcount):
    kind, offset, address, _, filesz, _, _, _ = struct.unpack_from('<IIIIIIII', data, phoff + i * phsize)
    if kind == 1 and filesz:
        cpu.mem_write(address, data[offset:offset + filesz])
cpu.reg_write(UC_ARM_REG_SP, 0x2001fff0)
symbols = {}
shoff, shsize, shcount = header[5], header[10], header[11]
sections = [struct.unpack_from('<IIIIIIIIII', data, shoff + i * shsize) for i in range(shcount)]
for section in sections:
    if section[1] != 2:
        continue
    strings = sections[section[6]]
    names = data[strings[4]:strings[4] + strings[5]]
    for offset in range(section[4], section[4] + section[5], section[9]):
        name, value, _, _, _, _ = struct.unpack_from('<IIIBBH', data, offset)
        end = names.find(b'\0', name)
        symbols[names[name:end].decode('ascii', errors='replace')] = value
stopped = False
def interrupt(uc, number, _):
    global stopped
    if number != 7:
        raise RuntimeError(f'Unexpected CPU interrupt {number} at {uc.reg_read(UC_ARM_REG_PC):08x}')
    stopped = True
    uc.emu_stop()
cpu.hook_add(UC_HOOK_INTR, interrupt)
started = time.monotonic()
cpu.emu_start(entry | 1, 0, timeout=240_000_000)
def word(name):
    return struct.unpack('<I', cpu.mem_read(symbols[name], 4))[0]
details = struct.unpack('<8f', cpu.mem_read(symbols['test_details'], 32)) if 'test_details' in symbols else ()
print(f'Cortex-M3 C tests: cases={word("test_case")}, elapsed={time.monotonic()-started:.1f}s')
if not stopped or not word('test_passed'):
    print(f'FAILED line={word("test_failed_line")} details={details}')
    sys.exit(1)
print('PASS: raw ADC1 identity, ordering, UART mode switching, retry and timeout recovery' if vofa_test else
      'PASS: timer/DMA bootstrap, dual mode restore, all rates and packed samples' if acquisition_test else
      'PASS: frequency, AC RMS, robust/model VPP, waveform, asynchronous HMI and dynamic-rate application pipeline')

if 'test_vpp_comparison' in symbols:
    values=struct.unpack('<3f',cpu.mem_read(symbols['test_vpp_comparison'],12))
    print('Synthetic 1 kHz / 1 Vpp at 3.3V reference (mV), 20k / 100k / 500k S/s:', ', '.join(f'{x:.3f}' for x in values))
if 'test_real_comparison' in symbols:
    rows=struct.unpack('<16f',cpu.mem_read(symbols['test_real_comparison'],64))
    for i,seg in enumerate((3407,3408)):
        hz,raw,fitted,valid,residual,selected,used,mv=rows[i*8:(i+1)*8]
        print(f'Recorded {seg}: Hz={hz:.6f}, robust={raw:.6f}, fitted={fitted:.6f}, '
              f'valid={valid:.0f}, residual={residual:.6f}, selected={selected:.6f}, '
              f'used_fit={used:.0f}, Vpp@3.19V={mv:.6f} mV')
if 'test_small_comparison' in symbols:
    values=struct.unpack('<4f',cpu.mem_read(symbols['test_small_comparison'],16))
    print('Synthetic noisy 50/100 mV, 20k/100k S/s cycle averages (mV):', ', '.join(f'{v:.3f}' for v in values))
if 'test_small_limits' in symbols:
    limits=struct.unpack('<8f',cpu.mem_read(symbols['test_small_limits'],32))
    print('Synthetic noisy sine phase-sweep ranges (mV):', ', '.join(f'{limits[i]:.3f}..{limits[i+1]:.3f}' for i in range(0,8,2)))
if 'test_small_pipeline_limits' in symbols:
    limits=struct.unpack('<4f',cpu.mem_read(symbols['test_small_pipeline_limits'],16))
    print('Simulated HMI 50/100 mV across probe cycles:', ', '.join(f'{limits[i]:.0f}..{limits[i+1]:.0f} mV' for i in (0,2)))
if 'test_large_pipeline_limits' in symbols:
    limits=struct.unpack('<2f',cpu.mem_read(symbols['test_large_pipeline_limits'],8))
    print(f'Simulated 1 V HMI with other-channel pickup: {limits[0]:.0f}..{limits[1]:.0f} mV')
