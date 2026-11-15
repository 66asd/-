"""Generate two persistent J-Link Keil projects and build AXF only."""
from pathlib import Path
import argparse
import copy
import os
import re
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MDK = ROOT / 'MDK-ARM'
DRIVER = 'Segger\\JL2CM3.dll'
VERSIONS = [('ADC_1k_20k', 16), ('ADC_1k_100k', 64)]


def generate():
    original = MDK / 'ADC.uvprojx'
    text = original.read_text(encoding='utf-8')
    text = re.sub(r'<CreateHexFile>\d</CreateHexFile>', '<CreateHexFile>0</CreateHexFile>', text)
    text = re.sub(r'<Flash2>.*?</Flash2>', lambda _: f'<Flash2>{DRIVER}</Flash2>', text)
    tree = ET.ElementTree(ET.fromstring(text))
    debug = tree.find('.//DebugOption')
    if debug.find('TargetDlls') is None:
        additions = []
        target = ET.SubElement(debug, 'Target')
        additions.append(target)
        for key in ['UseTarget', 'LoadApplicationAtStartup', 'RunToMain', 'RestoreBreakpoints',
                    'RestoreWatchpoints', 'RestoreMemoryDisplay', 'RestoreFunctions',
                    'RestoreToolbox', 'RestoreTracepoints']:
            ET.SubElement(target, key).text = '1'
        dlls = ET.SubElement(debug, 'TargetDlls')
        additions.append(dlls)
        for key, value in [('CpuDll', ''), ('CpuDllArguments', ''), ('PeripheralDll', ''),
                           ('PeripheralDllArguments', ''), ('InitializationFile', ''), ('Driver', DRIVER)]:
            ET.SubElement(dlls, key).text = value
        for item in additions:
            ET.indent(item, space='  ', level=5)
        inserted = '\n'.join('          '+ET.tostring(item,encoding='unicode') for item in additions)
        text = text.replace('        </DebugOption>',inserted+'\n        </DebugOption>')
    tree.find('.//TargetDlls/Driver').text = DRIVER
    original.write_text(text,encoding='utf-8')
    ET.indent(tree, space='  ')
    for name, points in VERSIONS:
        project_path = MDK / f'{name}.uvprojx'
        existed = project_path.exists()
        variant = ET.parse(project_path) if existed else copy.deepcopy(tree)
        previous_xml = ET.tostring(variant.getroot())
        defines = variant.find('.//Cads/VariousControls/Define')
        values = [v for v in defines.text.split(',') if not v.startswith('SCOPE_TARGET_POINTS=')]
        defines.text = ','.join(values + [f'SCOPE_TARGET_POINTS={points}'])
        variant.find('.//TargetCommonOption/OutputDirectory').text = f'..\\build\\{name}\\'
        variant.find('.//TargetCommonOption/OutputName').text = name
        variant.find('.//CreateHexFile').text = '0'
        variant.find('.//Utilities/Flash2').text = DRIVER
        original_app = next(g for g in tree.findall('.//Group') if g.findtext('GroupName')=='Scope Application')
        app = next(g for g in variant.findall('.//Group') if g.findtext('GroupName')=='Scope Application')
        if [p.text for p in app.findall('.//FilePath')] != [p.text for p in original_app.findall('.//FilePath')]:
            app.remove(app.find('Files'))
            app.append(copy.deepcopy(original_app.find('Files')))
        if not existed or ET.tostring(variant.getroot()) != previous_xml:
            variant.write(project_path, encoding='utf-8', xml_declaration=True)
        # Only debugger configuration; no window state, probe serial or breakpoints.
        # Preserve settings saved by Keil, including the user's selected probe.
        if (MDK / f'{name}.uvoptx').exists():
            continue
        opt = ET.Element('ProjectOpt')
        ET.SubElement(opt, 'SchemaVersion').text = '1.0'
        ET.SubElement(opt, 'Header').text = '### uVision Project, (C) Keil Software'
        t = ET.SubElement(opt, 'Target')
        ET.SubElement(t, 'TargetName').text = 'ADC'
        ET.SubElement(t, 'ToolsetNumber').text = '0x4'
        ET.SubElement(t, 'ToolsetName').text = 'ARM-ADS'
        options = ET.SubElement(t, 'TargetOption')
        ET.SubElement(ET.SubElement(options, 'OPTTT'), 'RunTarget').text = '1'
        d = ET.SubElement(options, 'DebugOpt')
        for key, value in [('uSim', '0'), ('uTrg', '1'), ('tLdApp', '1'), ('tGomain', '1'),
                           ('nTsel', '4'), ('pMon', DRIVER)]:
            ET.SubElement(d, key).text = value
        entry = ET.SubElement(ET.SubElement(options, 'TargetDriverDllRegistry'), 'SetRegEntry')
        ET.SubElement(entry, 'Number').text = '0'
        ET.SubElement(entry, 'Key').text = 'JL2CM3'
        ET.SubElement(entry, 'Name').text = (
            '-O78 -S2 -ZTIFSpeedSel1000 -A0 -C0 -JU1 -JI127.0.0.1 -JP0 -RST0 '
            '-N00("ARM CoreSight SW-DP") -D00(1BA01477) -L00(0) -TO18 -TC10000000 '
            '-TP21 -TDS8007 -TDT0 -TDC1F -TIEFFFFFFFF -TIP8 -TB1 -TFE0 -FO7 '
            '-FD20000000 -FC1000 -FN1 -FF0STM32F10x_128.FLM -FS08000000 -FL010000 '
            '-FP0($$Device:STM32F103C8$Flash\\STM32F10x_128.FLM)')
        ET.indent(opt, space='  ')
        ET.ElementTree(opt).write(MDK / f'{name}.uvoptx', encoding='utf-8', xml_declaration=True)


def build(path, log):
    uv = Path(os.environ.get('KEIL_ROOT', 'D:/keil5')) / 'UV4/UV4.exe'
    info = None
    if os.name == 'nt':
        info = subprocess.STARTUPINFO()
        info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        info.wShowWindow = subprocess.SW_HIDE
    process = subprocess.Popen([str(uv), '-b', str(path), '-o', str(log)], startupinfo=info)
    started = time.monotonic()
    while time.monotonic() - started < 90:
        if log.exists():
            text = log.read_text(encoding='utf-8', errors='replace')
            if 'Build Time Elapsed' in text:
                if '0 Error(s), 0 Warning(s)' not in text:
                    raise RuntimeError(text)
                print(text.strip())
                return
        time.sleep(0.3)
    raise RuntimeError(f'Keil build timeout; PID={process.pid}, log={log}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generate-only', action='store_true')
    args = parser.parse_args()
    generate()
    if not args.generate_only:
        out = ROOT / 'build/comparison'
        out.mkdir(parents=True, exist_ok=True)
        for name, _ in VERSIONS:
            build(MDK / f'{name}.uvprojx', out / f'{name}-{uuid.uuid4().hex[:12]}.txt')
            image = ROOT / 'build' / name / f'{name}.axf'
            assert image.exists(), f'Missing executable: {image}'
            print('Keil executable:', image)
