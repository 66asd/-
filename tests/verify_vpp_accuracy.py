"""Test 1000mVpp absolute algorithm error, including rejected-sine fallback.

Ground truth is the noise-free ADC-pin waveform. No empirical 1000/1030 gain.
--baseline saves measurements without asserting the improved accuracy targets.
"""
import sys
from pathlib import Path
loader=Path(__file__).with_name('verify_adc_scope.py').read_text()
exec(loader[:loader.index('rng=np.random.default_rng')])
rng=np.random.default_rng(10001030)
t=np.arange(1024)
dense=np.linspace(0,2*np.pi,262144,endpoint=False)
baseline='--baseline' in sys.argv
results=[]
for distortion,noise in [(0,3),(0,16),(.03,8),(.03,16),(.03,24),(.03,32)]:
    reference=np.sin(dense)+distortion*np.sin(3*dense+.4)
    scale=np.ptp(reference)
    values=[];methods=[];raw_values=[];window_counts=[];ready_counts=[]
    for frame in range(24 if baseline else 96):
        phase=rng.uniform(0,2*np.pi)
        angle=2*np.pi*t/64+phase
        clean=(np.sin(angle)+distortion*np.sin(3*angle+.4))/scale
        a=2048+clean*(1000*4096/3020)+rng.normal(0,noise,1024)
        b=2048+clean*(1000*4096/3020)+rng.normal(0,noise,1024)
        r,_=run(a,b,64000,auto=frame==0,publish=not baseline)
        raw_q8=struct.unpack('<2I',bytes(u.mem_read(symbols['measurement_vpp_q8'],8))) if not baseline else [0,0]
        if not baseline:
            raw_values.extend([v/256*3020/4096 for v in raw_q8])
            ready=struct.unpack('<2I',bytes(u.mem_read(symbols['vpp_ready'],8)))
            counts=struct.unpack('<2I',bytes(u.mem_read(symbols['vpp_window_frames'],8)))
            for ch in range(2):
                if ready[ch]:
                    values.append(int(r[ch,2]));ready_counts.append(counts[ch])
            window_counts.extend(counts)
        else:values.extend(r[:,2].tolist())
        methods.extend(struct.unpack('<2I',bytes(u.mem_read(symbols['sine_fit_used'],8))))
    assert values,('no converged VPP window',distortion,noise,window_counts[-2:])
    result={'harmonic_fraction':distortion,'noise_codes_rms':noise,
            'min_mv':min(values),'max_mv':max(values),'mean_mv':float(np.mean(values)),
            'sine_frames':sum(methods),'displayed_channel_frames':len(values),
            'first_ready_window_frames':min(ready_counts) if ready_counts else None}
    if raw_values:result['raw_range_mv']=[min(raw_values),max(raw_values)]
    results.append(result)
    if not baseline:
        assert max(abs(np.array(values)-1000))<=2,result
        assert abs(np.mean(values)-1000)<=1,result
if not baseline:
    # Exercise the same phase on every record (not just random-phase averaging),
    # and true voltages different from 1000mV. A hard-coded 1000/1030 fix fails here.
    other_levels=[]
    previous=None
    for target,distortion,voltage in [(1000,.03,3020),(500,0,3020),(750,0,3020),
                                      (1030,0,3020),(1250,0,3020),(1500,0,3020),
                                      (1000,0,2800),(1030,0,2800)]:
        reference=np.sin(dense)+distortion*np.sin(3*dense+.4)
        scale=np.ptp(reference);values=[]
        for frame in range(64):
            angle=2*np.pi*t/64+1.17
            clean=(np.sin(angle)+distortion*np.sin(3*angle+.4))/scale
            a=2048+clean*(target*4096/voltage)+rng.normal(0,8,1024)
            r,_=run(a,a,64000,vdda=voltage,auto=previous is None and frame==0,publish=True)
            if frame==0 and previous is not None:
                assert get('vpp_window_frames')==1,'real amplitude step was not reset'
                assert abs(int(r[0,2])-target)<=4,(target,r)
            if get('vpp_ready'):values.append(int(r[0,2]))
        assert values,(target,'no ready output')
        assert max(abs(np.array(values)-target))<=target*.002+1e-5,(target,values)
        other_levels.append({'target_mv':target,'vdda_mv':voltage,'min_mv':min(values),'max_mv':max(values)})
        previous=target
    print('Fixed-phase and amplitude-step checks:',json.dumps(other_levels))
print(json.dumps(results,indent=2))
(BUILD/('accuracy-baseline.json' if baseline else 'accuracy-current.json')).write_text(json.dumps(results,indent=2))
