"""Count executed Thumb instructions in the production image (not wall-clock cycles)."""
from pathlib import Path
from unicorn import UC_HOOK_BLOCK
loader = Path(__file__).with_name('verify_adc_scope.py').read_text()
exec(loader[:loader.index('rng=np.random.default_rng')])
counts, cache = [0], {}
def block(uc, address, size, data):
    key = (address, size)
    if key not in cache:
        code = bytes(uc.mem_read(address, size)); i = n = 0
        while i < len(code):
            half = int.from_bytes(code[i:i+2], 'little')
            i += 4 if half >> 11 >= 29 else 2
            n += 1
        cache[key] = n
    counts[0] += cache[key]
t = np.arange(1024)
a = 2048 + 365*np.sin(2*np.pi*t/180+.3)
u.hook_add(UC_HOOK_BLOCK, block)
run(a, a, 180000)
print(json.dumps({'dual_channel_measure_instructions': counts[0]}, indent=2))
(BUILD/'profile-current.json').write_text(json.dumps({'dual_channel_measure_instructions':counts[0]}))
