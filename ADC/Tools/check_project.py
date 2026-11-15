"""Read-only checks of CubeMX, Keil and the immutable HMI page3 contract."""
from pathlib import Path
import hashlib
import json
import re
import struct
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
HMI = ROOT.parent / 'DualDDS2.HMI'
EXPECTED_HMI_SHA256 = 'de7e13464afa0b9670ca8c2a4a6a086ffcf89c25d0a13b8a7b55f9a57f86ee07'

def archive_files(data):
    files = {}
    count = struct.unpack_from('<I', data)[0]
    assert 1 <= count <= 128, 'Unexpected HMI archive directory'
    for i in range(count):
        p = 4 + i * 28
        name = data[p:p+16].split(b'\0')[0]
        offset, size = struct.unpack_from('<II', data, p+16)
        if re.fullmatch(rb'[A-Za-z0-9_.]+', name) and offset + size <= len(data):
            files[name.decode()] = data[offset:offset+size]
    return files

def page_objects(data):
    objects = []
    for match in re.finditer(rb'att-(\d+)', data):
        p = match.end()
        attributes = {}
        for _ in range(int(match[1])):
            size = struct.unpack_from('<I', data, p)[0]
            p += 4
            assert 16 <= size <= 65536 and p+size <= len(data)
            key = data[p:p+16].split(b'\0')[0].decode('ascii')
            value = data[p+16:p+size]
            p += size
            attributes[key] = value
        events = {}
        while p+4 <= len(data):
            size = struct.unpack_from('<I', data, p)[0]
            p += 4
            if size == 0:
                break
            key = data[p:p+size]
            p += size
            event = re.fullmatch(rb'(codes[a-z]+)-(\d+)', key)
            if not event:
                break
            lines = []
            for _ in range(int(event[2])):
                size = struct.unpack_from('<I', data, p)[0]
                p += 4
                lines.append(data[p:p+size].decode('gb18030'))
                p += size
            events[event[1].decode()] = lines
        converted = {}
        for key, value in attributes.items():
            converted[key] = (value.decode('gb18030') if key in ('objname', 'txt')
                              else int.from_bytes(value, 'little') if len(value) <= 4 else value.hex())
        objects.append({'attributes': converted, 'events': events})
    return objects

data = HMI.read_bytes()
digest = hashlib.sha256(data).hexdigest()
assert digest == EXPECTED_HMI_SHA256, 'HMI changed: review the contract instead of guessing new controls'
files = archive_files(data)
assert b'baud=115200' in files['Program.s']
assert b'bkcmd=0' in files['Program.s']
objects = page_objects(files['3.pa'])
controls = {o['attributes']['objname']: o for o in objects}
assert len(controls) == 18 and controls['page3']['attributes']['id'] == 0
assert controls['page3']['events']['codesloadend'] == ['sendme']
expected = {'s0': (1,242,213), 's1': (15,239,211)}
config = (ROOT/'App/scope_config.h').read_text()
for channel, (name, (ident, width, height)) in enumerate(expected.items()):
    attrs = controls[name]['attributes']
    assert (attrs['id'], attrs['w'], attrs['h'], attrs['ch'], attrs['dis']) == (ident,width,height,1,100)
    letter = 'A' if channel == 0 else 'B'
    for suffix, value in [('WIDTH',width),('HEIGHT',height)]:
        assert re.search(rf'#define SCOPE_WAVE_{letter}_{suffix}\s+{value}u\b',config)
    assert re.search(rf'#define SCOPE_HMI_WAVE_{letter}_ID\s+{ident}u\b',config)
for name, ident in [('t1',3),('t3',5),('t5',7),('t7',9),('t9',11),('t11',13)]:
    attrs = controls[name]['attributes']
    assert attrs['id'] == ident and attrs['type'] == ord('t') and attrs['txt_maxl'] == 24
for name, ident in [('b1',16),('b2',17)]:
    assert controls[name]['attributes']['id'] == ident
    assert controls[name]['events']['codesup'] == [f'printh 65 03 {ident:02X} 00 FF FF FF']
ioc = dict(line.split('=',1) for line in (ROOT/'ADC.ioc').read_text().splitlines() if '=' in line)
for key, value in {
    'ADC1.Mode':'ADC_DUALMODE_REGSIMULT',
    'ADC1.ExternalTrigConv':'ADC_EXTERNALTRIGCONV_T3_TRGO',
    'Dma.ADC1.0.MemDataAlignment':'DMA_MDATAALIGN_WORD',
    'Dma.ADC1.0.PeriphDataAlignment':'DMA_PDATAALIGN_WORD',
    'Dma.ADC1.0.Mode':'DMA_CIRCULAR',
    'TIM3.TIM_MasterOutputTrigger':'TIM_TRGO_UPDATE',
}.items():
    assert ioc[key] == value, f'Incorrect CubeMX setting: {key}'
assert ioc['NVIC.USART1_IRQn'].startswith('true')
assert int(ioc['RCC.APB1TimFreq_Value']) // (int(ioc['TIM3.Prescaler'])+1) // (int(ioc['TIM3.Period'])+1) == 500000
for name, points in [('ADC',None),('ADC_1k_20k',16),('ADC_1k_100k',64)]:
    project = ET.parse(ROOT/f'MDK-ARM/{name}.uvprojx')
    assert project.find('.//CreateHexFile').text == '0', f'HEX output forbidden: {name}'
    assert project.find('.//Utilities/Flash2').text == 'Segger\\JL2CM3.dll'
    assert project.find('.//TargetDlls/Driver').text == 'Segger\\JL2CM3.dll'
    paths = [p.text.replace('\\','/') for p in project.findall('.//FilePath')]
    for source in sorted((ROOT/'App').glob('*.c')):
        assert paths.count('../App/'+source.name) == 1, f'{name} does not include {source.name}'
    for path in paths:
        assert (ROOT/'MDK-ARM'/path).exists(), f'Missing Keil source: {path}'
    assert '../App' in project.find('.//Cads/VariousControls/IncludePath').text.split(';')
    if points:
        assert f'SCOPE_TARGET_POINTS={points}' in project.find('.//Cads/VariousControls/Define').text.split(',')
        assert project.find('.//OutputDirectory').text == f'..\\build\\{name}\\'
        options = ET.parse(ROOT/f'MDK-ARM/{name}.uvoptx')
        assert options.find('.//DebugOpt/pMon').text == 'Segger\\JL2CM3.dll'
        assert options.find('.//SetRegEntry/Key').text == 'JL2CM3'
        assert '-FL010000' in options.find('.//SetRegEntry/Name').text
manifest = {'hmi_file':HMI.name, 'sha256':digest, 'baud':115200, 'page':3, 'objects':objects}
if '--write-manifest' in sys.argv:
    (ROOT/'hmi_page3_contract.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
else:
    stored = json.loads((ROOT/'hmi_page3_contract.json').read_text(encoding='utf-8'))
    assert stored == manifest, 'Stored HMI contract does not match the original file'
print('PASS: CubeMX dual ADC, three J-Link Keil projects with HEX disabled, immutable HMI/page3')
