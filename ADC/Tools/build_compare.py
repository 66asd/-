"""Build the normal low-rate firmware and a 1 kHz/100 kS/s comparison image."""
from pathlib import Path
import os
import shutil
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

root=Path(__file__).resolve().parents[1]
project=root/'MDK-ARM/ADC.uvprojx'
out=root/'build/comparison'
out.mkdir(parents=True,exist_ok=True)
uv=Path(os.environ.get('KEIL_ROOT','D:/keil5'))/'UV4/UV4.exe'

def build(path,log):
    started=time.monotonic()
    info=None
    if os.name=='nt':
        info=subprocess.STARTUPINFO()
        info.dwFlags|=subprocess.STARTF_USESHOWWINDOW
        info.wShowWindow=subprocess.SW_HIDE
    process=subprocess.Popen([str(uv),'-b',str(path),'-o',str(log)],startupinfo=info)
    while time.monotonic()-started<90:
        if log.exists():
            text=log.read_text(encoding='utf-8',errors='replace')
            if 'Build Time Elapsed' in text:
                if '0 Error(s), 0 Warning(s)' not in text:
                    raise RuntimeError(text)
                print(text.strip())
                return
        time.sleep(0.3)
    raise RuntimeError(f'Keil build timeout; PID={process.pid}, log={log}')

token=uuid.uuid4().hex[:12]
build(project,out/f'default-{token}.txt')
shutil.copyfile(root/'MDK-ARM/ADC/ADC.hex',out/'ADC_1k_20k.hex')
temp=project.parent/f'codex_compare_{token}.uvprojx'
try:
    tree=ET.parse(project)
    defines=tree.find('.//Cads/VariousControls/Define')
    defines.text=defines.text+',SCOPE_TARGET_POINTS=64'
    directory=out/'higher-rate'
    directory.mkdir(exist_ok=True)
    tree.find('.//TargetCommonOption/OutputDirectory').text=str(directory)+'\\'
    tree.find('.//TargetCommonOption/OutputName').text='ADC_1k_100k'
    tree.write(temp,encoding='utf-8',xml_declaration=True)
    build(temp,out/f'higher-{token}.txt')
    shutil.copyfile(directory/'ADC_1k_100k.hex',out/'ADC_1k_100k.hex')
finally:
    temp.unlink(missing_ok=True)
print('Images:',out/'ADC_1k_20k.hex',out/'ADC_1k_100k.hex',sep='\n')
