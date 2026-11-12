"""Execute the actual Cortex-M3 measurement code with Unicorn, not a Python port.
Requires a current Keil AXF build, numpy and unicorn.
"""
import json, math, pathlib, struct
import numpy as np
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_PC
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
def invoke(name,*args):
    u.reg_write(UC_ARM_REG_SP,0x2000fff0); u.reg_write(UC_ARM_REG_LR,STOP|1)
    for reg,arg in zip([UC_ARM_REG_R0,UC_ARM_REG_R1,UC_ARM_REG_R2,UC_ARM_REG_R3],args):u.reg_write(reg,int(arg))
    u.emu_start(symbols[name]|1,STOP,count=120000000)
    assert u.reg_read(UC_ARM_REG_PC)==STOP,('instruction budget',name,hex(u.reg_read(UC_ARM_REG_PC)))
    return u.reg_read(UC_ARM_REG_R0)
def put(name,value):u.mem_write(symbols[name],struct.pack('<I',value))
def get(name):return struct.unpack('<I',bytes(u.mem_read(symbols[name],4)))[0]
def run(a,b,fs,vdda=3020,auto=True,publish=False):
    p=np.rint(np.clip(a,0,4095)).astype(np.uint32) | (np.rint(np.clip(b,0,4095)).astype(np.uint32)<<16)
    u.mem_write(0x20010000,p.astype('<u4').tobytes());put('sample_hz',fs);put('vdda_mv',vdda)
    if auto:invoke('measure_auto')
    invoke('measure',0x20010000)
    if publish:invoke('measure_publish')
    r=np.array(struct.unpack('<12I',bytes(u.mem_read(symbols['reading'],48)))).reshape(2,6)
    trace=np.frombuffer(bytes(u.mem_read(symbols['trace'],482)),dtype=np.uint8).reshape(2,241)
    assert trace.min()>=17 and trace.max()<=193
    return r,trace
rng=np.random.default_rng(20261112); t=np.arange(1024);worst=np.zeros(3);frames=0
for fs,f in [(54000,300),(180000,1000)]+[(600000,f) for f in [10000,30000,39997,40000,45000,49999,60000,80000,99997,100000]]:
    for vpp in [0.5,1.0,2.5]:
        for phase in [0.07,1.3,3.9]:
            a=2048+vpp*1000*4096/3020/2*np.sin(2*np.pi*f*t/fs+phase)+rng.normal(0,3,len(t))
            b=2048+vpp*1000*4096/3020/2*np.sin(2*np.pi*f*t/fs+phase+0.73)+rng.normal(0,3,len(t))
            r,trace=run(a,b,fs)
            target=np.array([f,vpp*1000/(2*math.sqrt(2)),vpp*1000])
            error=np.max(abs(r[:,:3]/target-1),axis=0);worst=np.maximum(worst,error);frames+=1
            assert np.all(error<0.01),(fs,f,vpp,phase,r,error)
            if fs/f*4<1022:
                rises=np.sum((trace[:,:-1]<105)&(trace[:,1:]>=105),axis=1)
                assert np.all((rises>=3)&(rises<=5)),(f,rises)
# A held timebase changes the number of cycles; AUTO explicitly re-frames it.
a=2048+650*np.sin(2*np.pi*10000*t/600000)
run(a,a,600000)
a=2048+650*np.sin(2*np.pi*40000*t/600000)
r,trace=run(a,a,600000,auto=False)
assert np.sum((trace[0,:-1]<105)&(trace[0,1:]>=105))>10
r,trace=run(a,a,600000)
assert 3<=np.sum((trace[0,:-1]<105)&(trace[0,1:]>=105))<=5
for shape in ['square','triangle']:
    phase=(t/180+0.17)%1
    y=np.where(phase<0.5,1.,-1.) if shape=='square' else 4*abs(phase-0.5)-1
    r,_=run(2048+1000*y,2048+500*y,180000)
    target=np.array([1000,500])*3020/4096
    assert np.all(abs(r[:,2]/(2*target)-1)<0.015),(shape,r)
    assert np.all(abs(r[:,1]/(target if shape=='square' else target/math.sqrt(3))-1)<0.015),(shape,r)
r,_=run(np.full(1024,2048),np.full(1024,1200),600000)
assert np.all(r[:,:3]==0),r
r,_=run(2048+2500*np.sin(2*np.pi*t/180),2048+1000*np.sin(2*np.pi*t/180),180000)
assert r[0,5] and not r[1,5]
# Calibrated reference conversion uses the measured 3020mV anchor, not typical 1200mV.
assert invoke('reference_voltage',64*1630,64*1630,3020)==3020
assert abs(invoke('reference_voltage',64*1630,64*1492,3020)-3300)<=1
assert invoke('reference_voltage',64*1630,64*1758,3020)==2800
assert invoke('reference_voltage',64*1630,0,3020)==0
assert invoke('reference_voltage',64*1630,64*100,3020)==0
for voltage in [2700,2800,3020,3300]:
    a=2048+500*4096/voltage*np.sin(2*np.pi*40000*t/600000+0.3)
    r,_=run(a,a,600000,vdda=voltage)
    assert np.all(abs(r[:,2]-1000)<=3),(voltage,r)
# Run actual sample_start/sample_tick/sample_rate with peripheral HAL calls mocked.
# This verifies ordering/rollback; physical timing still needs a board/logic analyzer.
u.mem_map(0x40000000,0x30000)
ADC1=0x40012400;ADC2=0x40012800;TIM3=0x40000400;DMA=0x40020000;RCC=0x40021000;CAL=0x0800fc00
for sec in sections:
    if sec[1]==1 and sec[2]&2 and 0x08000000<=sec[3]<0x08100000:
        assert sec[3]+sec[5]<=CAL,'firmware overlaps calibration page'
def write32(addr,value):u.mem_write(addr,struct.pack('<I',value))
def read32(addr):return struct.unpack('<I',bytes(u.mem_read(addr,4)))[0]
for name,addr in [('hadc1',ADC1),('hadc2',ADC2),('htim3',TIM3)]:write32(symbols[name],addr)
write32(ADC1+8,0x180000);write32(ADC1+12,0x12345);write32(ADC1+52,0)
write32(RCC+4,0x400);write32(TIM3+44,119)
u.mem_write(CAL,b'\xff'*1024)
mock={'running':False,'dma':False,'code':1630,'read':0,'timeout':False,'writes':0,'events':[]}
def hal(uc,addr,size,name):
    a=uc.reg_read(UC_ARM_REG_R0);b=uc.reg_read(UC_ARM_REG_R1);c=uc.reg_read(UC_ARM_REG_R2)
    ret=0;mock['events'].append(name)
    if name=='Error_Handler':raise AssertionError('firmware Error_Handler')
    if name=='HAL_RCC_GetPCLK1Freq':ret=36000000
    elif name in ['HAL_ADCEx_Calibration_Start','HAL_ADC_Start']:
        # STM32F1 HAL calibration returns with ADON set; it is not a no-op.
        adc=read32(a);write32(adc+8,read32(adc+8)|1)
    elif name=='HAL_TIM_Base_Stop':mock['running']=False
    elif name=='HAL_ADCEx_MultiModeStop_DMA':
        assert not mock['running'];mock['dma']=False;write32(ADC1+4,read32(ADC1+4)&~0xf0000)
        for adc in [ADC1,ADC2]:write32(adc+8,read32(adc+8)&~1)
    elif name=='HAL_ADC_ConfigChannel':
        assert not mock['running'] and not mock['dma'],'reference disturbed running external ADC'
        channel,rank,sampling=struct.unpack('<3I',bytes(u.mem_read(b,12)))
        assert (channel,rank,sampling)==(17,1,7)
        write32(ADC1+52,17);write32(ADC1+12,7<<21);mock['read']=0
    elif name=='HAL_ADC_PollForConversion':ret=3 if mock['timeout'] else 0
    elif name=='HAL_ADC_GetValue':
        mock['read']+=1
        ret=[100,4095][mock['read']-1] if mock['read']<=2 else mock['code']
    elif name=='HAL_ADCEx_MultiModeStart_DMA':
        assert not mock['running'];assert read32(ADC1+4)&0xf0000==0x60000
        assert c==2048;mock['dma']=True;write32(DMA+12,2048)
        for adc in [ADC1,ADC2]:write32(adc+8,read32(adc+8)|1)
    elif name=='HAL_TIM_Base_Start':
        assert mock['dma'] and read32(ADC1+52)==0 and read32(ADC1+12)==0x12345
        mock['running']=True;mock['produced']=0
    elif name=='HAL_FLASH_Program':
        assert not mock['running'] and not mock['dma'] and a==2
        assert CAL<=b<CAL+16 and read32(b)==0xffffffff
        write32(b,c);mock['writes']+=1
    uc.reg_write(UC_ARM_REG_R0,ret);uc.reg_write(UC_ARM_REG_PC,uc.reg_read(UC_ARM_REG_LR))
hal_names=['HAL_ADCEx_Calibration_Start','HAL_Delay','HAL_ADC_Start','HAL_ADC_ConfigChannel','HAL_ADC_PollForConversion','HAL_ADC_GetValue','HAL_ADCEx_MultiModeStart_DMA','HAL_ADCEx_MultiModeStop_DMA','HAL_TIM_Base_Start','HAL_TIM_Base_Stop','HAL_FLASH_Unlock','HAL_FLASH_Lock','HAL_FLASH_Program','HAL_RCC_GetPCLK1Freq','Error_Handler']
for name in hal_names:
    address=symbols[name]&~1
    u.hook_add(UC_HOOK_CODE,hal,user_data=name,begin=address,end=address)
put('uwTick',0);put('sample_hz',600000);invoke('sample_start')
assert get('vdda_mv')==3020 and mock['writes']==4 and mock['running']
assert read32(CAL)==0x56445231 and read32(CAL+4)==64*1630
put('uwTick',4999);events=len(mock['events']);invoke('sample_tick');assert len(mock['events'])==events
mock['code']=1492;put('uwTick',5000);invoke('sample_tick')
assert abs(get('vdda_mv')-3300)<=1 and mock['writes']==4 and mock['running']
assert get('reference_updates')==1
mock['code']=1758;put('uwTick',10000);invoke('sample_tick')
assert get('vdda_mv')==2800 and get('reference_updates')==2 and read32(CAL+8)==3020
prior=get('vdda_mv');mock['timeout']=True;put('uwTick',15000);invoke('sample_tick')
assert get('vdda_mv')==prior and get('reference_errors')==1 and mock['running']
mock['timeout']=False;mock['code']=0;put('uwTick',20000);invoke('sample_tick')
assert get('vdda_mv')==prior and get('reference_errors')==2 and mock['running']
mock['code']=1758;mock['running']=False;mock['dma']=False
invoke('sample_start') # Reboot uses stored calibration; no re-anchoring to3020.
assert get('vdda_mv')==2800 and mock['writes']==4
assert invoke('sample_rate',1)==1 and get('sample_hz')==2000 and read32(TIM3+44)==35999
assert get('reference_next')==get('uwTick')+5000,'slow frames starved by reference updates'
assert invoke('sample_rate',9999999)==1 and get('sample_hz')==600000 and read32(TIM3+44)==119
# The first two post-switch records are discarded, never spliced into an old frame.
for index,(cb,left) in enumerate([('HAL_ADC_ConvHalfCpltCallback',1024),('HAL_ADC_ConvCpltCallback',2048),('HAL_ADC_ConvHalfCpltCallback',1024)]):
    write32(DMA+12,left);invoke(cb,symbols['hadc1']);ptr=invoke('sample_take')
    assert bool(ptr)==(index==2),(index,ptr)
# A low-rate valid alias is forcibly probed again at maximum bandwidth.
put('uwTick',5000);put('sample_hz',54000)
u.mem_write(symbols['page'],struct.pack('<I',3));write32(symbols['paint'],0)
u.mem_write(symbols['scope_hold'],b'\0');write32(symbols['txlen'],0)
u.mem_write(symbols['scope_clear'],b'\0');u.mem_write(symbols['scope_auto'],b'\1')
write32(symbols['query'],6000);write32(symbols['nextprobe'],4999)
invoke('screen_tick');assert get('sample_hz')==600000
# Exercise the whole main-loop path over sixteen simulated seconds, across VREF
# pauses and a 1kHz -> 50kHz input jump. UART and DMA registers are modelled;
# screen_tick, sample_tick, callbacks, parser and waveform serialization are real ARM.
mock['running']=False;mock['dma']=False;mock['code']=1630;put('uwTick',6000)
invoke('sample_start');invoke('scope_key',16)
write32(symbols['page'],3);write32(symbols['query'],30000)
USART=0x40013800;write32(USART,0x80)
uart=bytearray();seen_hz=[];early_vpp=[];early_cycles=[];updates_before=get('reference_updates')
command=bytearray();commands=[];payloads=[];raw_remaining=0;raw=bytearray();acks=[]
restarted_mid_payload=False;hold_count=None
for now in range(6000,22000,2):
    put('uwTick',now)
    physical_mv=3020 if now<11000 else 2800
    mock['code']=1630 if now<11000 else 1758
    for due, packet in list(acks):
        if due<=now:
            for byte in packet:invoke('receive',byte)
            acks.remove((due,packet))
    if now==17000:invoke('scope_key',17)
    if now==17200:hold_count=get('screen_frames')
    if now==17398:assert get('screen_frames')==hold_count,'STOP did not freeze'
    if now==17400:invoke('scope_key',16)
    if not restarted_mid_payload and now>6600 and get('burst_state')==3 and get('txlen'):
        invoke('scope_key',16);restarted_mid_payload=True
    if mock['running']:
        fs=get('sample_hz');before=mock['produced'];after=before+fs*2/1000
        mock['produced']=after
        f=1000 if now<14000 else 50000
        basis=int(after)//2048*2048
        times=(np.arange(2048)+basis)/fs
        a=np.rint(2048+500*4096/physical_mv*np.sin(2*np.pi*f*times+0.07)).astype(np.uint32)
        b=np.rint(2048+500*4096/physical_mv*np.sin(2*np.pi*f*times+0.8)).astype(np.uint32)
        u.mem_write(symbols['dma'],(a|(b<<16)).astype('<u4').tobytes())
        for boundary in range(int(before)//1024+1,int(after)//1024+1):
            invoke('HAL_ADC_ConvHalfCpltCallback' if boundary%2 else 'HAL_ADC_ConvCpltCallback',symbols['hadc1'])
        write32(DMA+12,2048-int(after)%2048)
    invoke('sample_tick')
    oldframes=get('screen_frames');invoke('screen_tick')
    if get('screen_frames')>oldframes:
        seen_hz.append(get('reading'))
        if now<14000:
            early_vpp.append(read32(symbols['reading']+8))
            early_cycles.append(get('measurement_cycles'))
    # The UART gets a real 115200-baud byte budget, not an instantaneous flush.
    budget=23
    while get('txlen') and budget:
        oldpos=get('txpos');invoke('screen_uart_irq')
        if get('txpos')<=oldpos:continue
        budget-=1;byte=read32(USART+4)&255;uart.append(byte)
        if raw_remaining:
            raw.append(byte);raw_remaining-=1
            if not raw_remaining:
                assert min(raw)>=17 and max(raw)<=193
                payloads.append((channel,bytes(raw)));raw.clear()
                acks.append((now+2,b'\xfd\xff\xff\xff'))
        else:
            command.append(byte)
            if command.endswith(b'\xff\xff\xff'):
                cmd=bytes(command[:-3]);command.clear();commands.append(cmd)
                if cmd.startswith(b'addt '):
                    channel,zero,raw_remaining=map(int,cmd[5:].split(b','))
                    assert zero==0 and (channel,raw_remaining) in [(1,241),(15,238)],cmd
                    acks.append((now+6,b'\xfe\xff\xff\xff'))
                # Successful ASCII commands can still return 01 while FE is pending.
                acks.append((now+2,b'\x01\xff\xff\xff'))
screen_frames=get('screen_frames')
assert screen_frames>=70 and any(abs(h-1000)<10 for h in seen_hz) and abs(seen_hz[-1]-50000)<500,seen_hz
assert len(payloads)>=2*screen_frames and not any(c.startswith(b'add ') for c in commands)
assert restarted_mid_payload and get('screen_burst_errors')==0
assert early_cycles and min(early_cycles)>=4,early_cycles
assert min(early_vpp)>=997 and max(early_vpp)<=1003,early_vpp
assert get('reference_updates')-updates_before>=3
assert get('vdda_mv')==2800 and read32(CAL+8)==3020
assert b't7.txt="2800mV"' in commands,'screen did not show the automatically updated reference'
# Recover from missing transparent-mode acknowledgements without leaving the panel
# permanently waiting for raw bytes. Also exercise the older-panel ASCII fallback.
put('uwTick',23000);write32(symbols['txlen'],0);write32(symbols['paint'],1)
write32(symbols['page'],3);write32(symbols['query'],30000)
write32(symbols['plot'],8);write32(symbols['burst_channel'],0)
write32(symbols['burst_state'],1);write32(symbols['burst_deadline'],22999)
invoke('screen_tick')
assert get('burst_state')==3 and get('txlen')==241,'lost FE did not complete raw byte count'
sent=bytearray()
while get('txlen'):
    pos=get('txpos');invoke('screen_uart_irq')
    if get('txpos')>pos:sent.append(read32(USART+4)&255)
assert sent==bytes(u.mem_read(symbols['trace'],241))
for byte in b'\xfd\xff\xff\xff':invoke('receive',byte)
invoke('screen_tick');assert get('burst_state')==0 and get('plot')==9
write32(symbols['burst_state'],3);write32(symbols['burst_deadline'],22999)
invoke('screen_tick')
assert get('burst_disabled') and get('plot')==8 and get('txlen')==3
while get('txlen'):invoke('screen_uart_irq')
invoke('screen_tick')
assert bytes(u.mem_read(symbols['tx'],get('txlen'))).startswith(b'add 1,0,'),'ASCII fallback failed'
result={'sine_frames':frames,'max_relative_error':dict(zip(['frequency','AC_RMS','VPP'],worst.tolist())),'checks':['300Hz-100kHz sine','square/triangle','DC/clipping','held timebase and AUTO','VREF calibrated ratio and invalid codes','startup calibration persisted once','scheduled VREF stop/restore ordering','reference timeout and bad code recovery','timer divider bounds','DMA transition frame rejection','alias escape probe']}
result['checks']+=['real HAL post-calibration ADON guard','continuous DMA-to-UART output across VREF pauses',
                  '115200-baud addt FE/FD transport and exact byte counts',
                  'STOP and AUTO during raw transfer','no short-probe VPP publication',
                  'lost FE/FD recovery and ASCII fallback','five-second VREF deadline survives rate probes']
result['checks']+=['VDDA auto tracks 3020mV to 2800mV without changing initial calibration']
result['stream_test']={'screen_frames':screen_frames,'first_and_last_hz':[seen_hz[0],seen_hz[-1]],
                       'uart_bytes':len(uart),'raw_blocks':len(payloads),
                       '1kHz_vpp_range_mv':[min(early_vpp),max(early_vpp)],
                       '1kHz_window_cycles':[min(early_cycles),max(early_cycles)]}
print(json.dumps(result,indent=2))
(BUILD/'results.json').write_text(json.dumps(result,indent=2))

