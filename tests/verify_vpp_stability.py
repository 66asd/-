"""Multi-frame VPP stability and amplitude-step tests of the production ARM binary.

--baseline records old behavior without asserting the new stability requirements.
Noise is synthetic; these results do not replace measurements on the actual board.
"""
import sys
from pathlib import Path

loader = Path(__file__).with_name('verify_adc_scope.py').read_text()
exec(loader[:loader.index('rng=np.random.default_rng')])
baseline = '--baseline' in sys.argv
rng = np.random.default_rng(5382026)
t = np.arange(1024)
results = []
for fs, frequency in [(180000, 1000), (64000, 1000), (600000, 40000), (600000, 99997)]:
    values, fits = [], []
    for frame in range(32):
        phase = rng.uniform(-np.pi, np.pi)
        clean = 2048 + (538 * 4096 / 3020 / 2) * np.sin(2*np.pi*frequency*t/fs+phase)
        a = clean + rng.normal(0, 8, 1024)
        b = clean + rng.normal(0, 8, 1024)
        # Sparse interference, independent of signal phase; no input clipping.
        a[rng.choice(1024, 3, replace=False)] += rng.choice([-80, 80], 3)
        b[rng.choice(1024, 3, replace=False)] += rng.choice([-80, 80], 3)
        r, _ = run(a, b, fs)
        values.extend(r[:, 2].tolist())
        if not baseline:
            flags = struct.unpack('<2I', bytes(u.mem_read(symbols['sine_fit_used'], 8)))
            fits.extend(flags)
            assert all(flags), (frequency, frame, r, flags)
            assert np.all(abs(r[:, 0]/frequency-1) < .002), r
    metric = {'frequency': frequency, 'sample_hz': fs, 'frames_per_channel': 32,
              'min_mv': min(values), 'max_mv': max(values),
              'mean_mv': float(np.mean(values)), 'std_mv': float(np.std(values))}
    results.append(metric)
    if not baseline:
        assert max(values)-min(values) <= 5, metric
        assert max(abs(np.array(values)/538-1)) < .01, metric
        assert abs(np.mean(values)/538-1) < .003, metric

if not baseline:
    # Exact quantile ranks: cover equal values, shared/different coarse bins and rails.
    for data in [np.full(1024, 2048), np.arange(1024)+1500,
                 rng.integers(0, 4096, 1024), rng.integers(2016, 2048, 1024)]:
        raw = data.astype(np.uint32)
        u.mem_write(0x20010000, (raw | raw << 16).astype('<u4').tobytes())
        expected = np.sort(raw)[1018] - np.sort(raw)[5]
        for channel in [0, 1]:
            bits = invoke('quantile_span', 0x20010000, channel)
            actual = struct.unpack('<f', struct.pack('<I', bits))[0]
            assert actual == expected, (actual, expected)
    # No cross-frame smoothing: a real amplitude step must appear in its FIRST frame.
    steps = []
    for amplitude in [538, 538, 538, 800, 800, 300, 538]:
        a = 1900 + amplitude*4096/3020/2*np.sin(2*np.pi*t/180+.63)
        a += rng.normal(0, 8, 1024)
        r, _ = run(a, a, 180000)
        assert np.all(abs(r[:, 2]-amplitude) <= 3), (amplitude, r)
        steps.append(int(r[0, 2]))
    # Harmonics must not be silently discarded by treating all periodic waves as sine.
    for harmonic in [2, 3, 4, 5]:
        phase = 2*np.pi*t/180+.31
        y = np.sin(phase) + .05*np.sin(harmonic*phase+.4)
        a = 2048 + 700*y + rng.normal(0, 2, 1024)
        r, _ = run(a, a, 180000)
        assert not any(struct.unpack('<2I', bytes(u.mem_read(symbols['sine_fit_used'], 8)))), harmonic
        dense = np.linspace(0, 2*np.pi, 100000)
        waveform = np.sin(dense)+.05*np.sin(harmonic*dense+.4)
        expected = np.ptp(waveform)*700*3020/4096
        assert np.all(abs(r[:, 2]/expected-1) < .01), (harmonic, expected, r)
    print('Amplitude steps, first-frame VPP (mV):', steps)
print(json.dumps(results, indent=2))
(BUILD/('vpp-baseline.json' if baseline else 'vpp-stability.json')).write_text(json.dumps(results, indent=2))
