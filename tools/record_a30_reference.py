"""Run the supplied A3 GUI unchanged, recording its CRC-checked uplink frames.

No automatic connect, homing, enable or motion commands. Operate the original
GUI normally; the extra JSONL can be opened directly in our offline viewer.
"""
import argparse
import json
from pathlib import Path
import runpy
import threading
import time
from datetime import datetime
import hashlib


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',type=Path,required=True,help='原工具完整路径；同目录需保留 H7_USB_OTA.pyw')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--hand-side',choices=['left','right'],default='left')
    args=parser.parse_args()
    reference=args.reference.resolve()
    scope=runpy.run_path(str(reference),run_name='a30_reference_recorder')
    directory=Path(__file__).resolve().parents[1]/'logs'
    output=args.output or directory/f'hand_reference_{datetime.now():%Y%m%d_%H%M%S}.jsonl'
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x',encoding='utf-8') as stream:
        started=time.perf_counter();seq=0;lock=threading.Lock();capture_failed=False
        def record(kind,**data):
            nonlocal seq
            with lock:
                seq+=1
                stream.write(json.dumps(dict(type=kind,seq=seq,t=time.perf_counter()-started,
                    hand_side=args.hand_side,**data),ensure_ascii=False)+'\n')
                stream.flush()
        record('session',schema='hand-control-session/v2',metadata={
            'model_id':'TsenseI_v3_0','hand_side':args.hand_side,
            'source':'A3_USB_Downlink reference GUI CRC-checked uplink',
            'reference_file':str(reference),'reference_sha256':hashlib.sha256(reference.read_bytes()).hexdigest()})
        globals_=scope['A3UsbApp']._rx_loop.__globals__
        original=globals_['pop_aa55_frames']
        def capture(buffer,*positional,**keywords):
            nonlocal capture_failed
            frames=original(buffer,*positional,**keywords)
            for frame in frames:
                if capture_failed or len(frame)<8 or frame[4]!=3 or frame[5] not in (2,3):continue
                payload=frame[6:-2]
                try:
                    record('telemetry' if frame[5]==2 else 'board_telemetry',
                        model_id='TsenseI_v3_0',payload_hex=payload.hex(),wire_hex=frame.hex(),
                        accepted=True,crc_verified=True)
                except OSError as error:
                    capture_failed=True
                    print(f'记录失败，原工具继续运行：{error}',flush=True)
            return frames
        globals_['pop_aa55_frames']=capture
        print(f'原工具上行记录：{output.resolve()}',flush=True)
        try:
            scope['A3UsbApp']().mainloop()
        finally:
            globals_['pop_aa55_frames']=original
            if not capture_failed:record('session_end',reason='reference_gui_closed')


if __name__=='__main__':
    main()
