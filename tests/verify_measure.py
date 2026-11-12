"""Execute the actual Cortex-M3 measurement code with Unicorn, not a Python port.
Requires a current Keil AXF build, numpy and unicorn.
"""
import json, math, pathlib, struct
import numpy as np
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC
ROOT = pathlib.Path(__file__).resolve().parents[1]
BUILD = ROOT / 'build' / 'measurement-test'
BUILD.mkdir(parents=True, exist_ok=True)
# Parse the production Keil ELF32 image and its symbol table.
elf=(ROOT/'MDK-ARM/DualDDS_SPI/DualDDS_SPI.axf').read_bytes()
header=struct.unpack_from('<16sHHIIIIIHHHHHH',elf)
assert header[0][:4]==b'\x7fELF' and header[0][4:6]==b'\x01\x01'
phoff,shoff=header[5:7]; phsize,phnum,shsize,shnum=header[9:13]
sections=[struct.unpack_from('<10I',elf,shoff+i*shsize) for i in range(shnum)]
symbols={}
for sec in sections:
    if sec[1]!=2:continue
    strings=sections[sec[6]]
    strings=elf[strings[4]:strings[4]+strings[5]]
    for off in range(sec[4],sec[4]+sec[5],sec[9]):
        name,value,size,info,other,ndx=struct.unpack_from('<IIIBBH',elf,off)
        end=strings.find(b'\0',name)
        symbols[strings[name:end].decode(errors='replace')]=value
u=Uc(UC_ARCH_ARM,UC_MODE_THUMB|UC_MODE_MCLASS)
u.mem_map(0x08000000,0x100000); u.mem_map(0x20000000,0x20000)
for i in range(phnum):
    typ,off,va,pa,filesz,memsz,flags,align=struct.unpack_from('<8I',elf,phoff+i*phsize)
    if typ==1 and filesz:u.mem_write(va,elf[off:off+filesz])
STOP=0x080ff000
u.hook_add(UC_HOOK_CODE,lambda uc,a,s,d: uc.emu_stop(),begin=STOP,end=STOP)
def run(a,b,fs):
    p=np.rint(np.clip(a,0,4095)).astype(np.uint32) | (np.rint(np.clip(b,0,4095)).astype(np.uint32)<<16)
    u.mem_write(0x20010000,p.astype('<u4').tobytes())
    u.mem_write(symbols['sample_hz'],struct.pack('<I',fs)); u.mem_write(symbols['vdda_mv'],struct.pack('<I',3300))
    u.reg_write(UC_ARM_REG_SP,0x2000fff0);u.reg_write(UC_ARM_REG_LR,STOP|1);u.reg_write(UC_ARM_REG_R0,0x20010000)
    u.emu_start(symbols['measure']|1,STOP,count=120000000)
    assert u.reg_read(UC_ARM_REG_PC)==STOP, ('instruction budget exhausted',hex(u.reg_read(UC_ARM_REG_PC)))
    readings=np.array(struct.unpack('<12I',bytes(u.mem_read(symbols['reading'],48)))).reshape(2,6)
    trace=np.frombuffer(bytes(u.mem_read(symbols['trace'],482)),dtype=np.uint8).reshape(2,241)
    assert trace.min()>=17 and trace.max()<=193
    return readings,trace
rng=np.random.default_rng(20261112); t=np.arange(1024)
worst=np.zeros(3); rows=[]
for fs,f in [(400000,300),(54000,300),(180000,1000),(400000,2345),(400000,5000),(400000,9997),(400000,10000)]:
    for vpp in [0.5,1.0,2.5]:
        for phase in [0.07,1.3,3.9]:
            a=2048+vpp*4096/3300*500*np.sin(2*np.pi*f*t/fs+phase)
            a+=rng.normal(0,3,len(t))
            b=2048+vpp*4096/3300*500*np.sin(2*np.pi*f*t/fs+phase+0.73)
            b+=rng.normal(0,3,len(t))
            readings,trace=run(a,b,fs)
            expected=np.array([f,vpp*1000/(2*math.sqrt(2)),vpp*1000])
            error=np.max(abs(readings[:,:3]/expected-1),axis=0)
            # Initial search at 400k/300Hz may contain less than two rising edges.
            if np.all(readings[:,0]):
                worst=np.maximum(worst,error)
                assert np.all(error<0.01), (fs,f,vpp,phase,readings,error)
                if fs/f*4 < 1022:
                    rises=np.sum((trace[:,:-1]<105)&(trace[:,1:]>=105),axis=1)
                    assert np.all((rises>=3)&(rises<=5)),rises
            rows.append([fs,f,vpp,phase,error.tolist()])
for shape in ['square','triangle']:
    f=1000;fs=180000;phase=(t*f/fs+0.17)%1
    y=np.where(phase<0.5,1.,-1.) if shape=='square' else 4*abs(phase-0.5)-1
    readings,_=run(2048+1000*y,2048+500*y,fs)
    target=np.array([1000*3300/4096,500*3300/4096])
    assert np.all(abs(readings[:,2]/(2*target)-1)<0.015),(shape,readings)
    target_rms=target if shape=='square' else target/math.sqrt(3)
    assert np.all(abs(readings[:,1]/target_rms-1)<0.015),(shape,readings)
r,_=run(np.full(1024,2048),np.full(1024,1200),400000)
assert np.all(r[:,:3]==0),r
r,_=run(2048+2500*np.sin(2*np.pi*t/180),2048+1000*np.sin(2*np.pi*t/180),180000)
assert r[0,5] and not r[1,5],r
def invoke(name,arg):
    u.reg_write(UC_ARM_REG_SP,0x2000fff0)
    u.reg_write(UC_ARM_REG_LR,STOP|1)
    u.reg_write(UC_ARM_REG_R0,arg)
    u.emu_start(symbols[name]|1,STOP,count=1000000)
    assert u.reg_read(UC_ARM_REG_PC)==STOP
def touch(component,event=0):
    for byte in [0x65,3,component,event,255,255,255]:invoke('receive',byte)
u.mem_write(symbols['scope_hold'],b'\0')
touch(17,1)  # Press is ignored, release is handled once.
assert bytes(u.mem_read(symbols['scope_hold'],1))==b'\0'
touch(17)
assert bytes(u.mem_read(symbols['scope_hold'],1))==b'\1'
touch(15)  # Touching waveform s1 must not resume acquisition.
assert bytes(u.mem_read(symbols['scope_hold'],1))==b'\1'
touch(16)
assert bytes(u.mem_read(symbols['scope_hold'],1))==b'\0'
assert bytes(u.mem_read(symbols['scope_clear'],1))==b'\2'
# Read the current serialized page3 (the HMI also contains an older copy).
blob=(ROOT/'DualDDS2.HMI').read_bytes()
import re
objects=list(re.finditer(b'objname'+b'\0'*9,blob))
page_start=max(m.start() for m in objects if blob[m.end():m.end()+5]==b'page3')
page_objects={}
for m in objects:
    if m.start()<page_start:continue
    name=re.match(rb'[A-Za-z0-9_]+',blob[m.end():]).group().decode()
    if name.startswith('page') and m.start()!=page_start:break
    at=blob.rfind(b'id\0',m.start()-40,m.start())
    page_objects[name]=(blob[at+16],m.start())
    if name=='b2':break
for name,expected in [('s0',1),('s1',15),('b1',16),('b2',17)]:
    assert page_objects[name][0]==expected,(name,page_objects[name])
for name in ['b1','b2']:
    at=blob.index(b'sendkey'+b'\0'*9,page_objects[name][1])
    assert blob[at+16]&2,'HMI release reporting disabled'
result={'sine_frames':len(rows),'max_relative_error':dict(zip(['frequency','AC_RMS','VPP'],worst.tolist())),'other_checks':['square','triangle','DC','clipping','trace bounds']}
result['other_checks']+=['3-5 visible cycles','actual HMI IDs and release events','ARM UART parser AUTO/STOP']
(BUILD/'results.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result,indent=2))



