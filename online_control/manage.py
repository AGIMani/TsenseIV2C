"""Deploy/restart/status both node0 web services from this working tree."""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
REMOTE = '/home/houjue/online_control'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['deploy', 'restart', 'status'])
    parser.add_argument('--only', choices=['all', 'play', 'track'], default='all')
    parser.add_argument('--target', default='houjue@node0')
    parser.add_argument('--no-pause', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.@-]*',args.target):
        parser.error('Invalid SSH destination')
    native = Path(os.environ.get('SystemRoot','C:/Windows'))/'System32/OpenSSH/ssh.exe'
    ssh = str(native) if sys.platform=='win32' else shutil.which('ssh')
    def remote(command, text=None):
        subprocess.run([ssh,'-o','BatchMode=yes','-o','ConnectTimeout=10',args.target,command],
                       input=text,text=True,encoding='utf-8',check=True,cwd=ROOT)
    kinds = ['track','play'] if args.only=='all' else [args.only]
    if args.action == 'deploy':
        # Move installations without losing data, CA, history, or existing scripts.
        # Existing absolute bind-mount/current paths remain valid through aliases.
        remote('python3 -', '''from pathlib import Path
home=Path('/home/houjue')
base=home/'online_control'
base.mkdir(exist_ok=True)
for old_name,new_name in [('tsensei-hand-web','online_motion_track'),('online-motion-play','online_motion_play')]:
 old,new=home/old_name,base/new_name
 if old.is_symlink():
  if old.resolve()!=new.resolve(): raise RuntimeError('Unexpected installation alias: '+str(old))
  continue
 if old.exists():
  if new.exists(): raise RuntimeError('Both old and new installations exist')
  if not old.is_dir() or not (old/'current').is_dir(): raise RuntimeError('Invalid installation')
  old.rename(new)
  old.symlink_to(new,target_is_directory=True)
 elif not new.exists():
  raise RuntimeError('Existing node0 installation missing: '+str(old))
print('Installations and persistent data: '+str(base))
''')
        for kind in kinds:
            entry = HERE/'online_motion_play/deploy_node0.py' if kind=='play' else HERE/'online_motion_track/deployment/node0/deploy_node0.py'
            subprocess.run([sys.executable,'-X','utf8',str(entry),'--target',args.target],check=True,cwd=ROOT)
        scp = str(native.with_name('scp.exe')) if sys.platform=='win32' else shutil.which('scp')
        subprocess.run([scp,'-S',ssh,'-o','BatchMode=yes','-o','ConnectTimeout=10',
                        'online_cmd.md',f'{args.target}:{REMOTE}/online_cmd.md'],check=True,cwd=ROOT)
    else:
        for kind in kinds:
            folder = REMOTE + '/online_motion_' + kind + '/current'
            command = 'restart app gateway' if args.action=='restart' else 'ps'
            remote(f'cd {folder} && docker compose {command}')
    if args.action != 'status':
        remote("python3 -", '''import json,urllib.request,time
values=[]
for port in (8766,8769):
 request=urllib.request.Request(f'http://127.0.0.1:{port}/api/health',headers={'Host':f'127.0.0.1:{8767 if port==8769 else port}'})
 for attempt in range(30):
  try:
   with urllib.request.urlopen(request,timeout=3) as r: values.append(json.load(r))
   break
  except OSError:
   if attempt==29: raise
   time.sleep(1)
assert values[0]['downlink_sha256']==values[1]['downlink_sha256'], 'Shared downlink versions differ'
print(json.dumps(values,ensure_ascii=False,indent=2))
''')
    print('Motion: https://192.168.88.229:8767/\nTracking: https://192.168.88.229/')


if __name__=='__main__':
    main()
