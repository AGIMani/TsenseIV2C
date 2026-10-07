"""Exercise identical public HTTP actions against old and new playback backends.
Never opens a serial port or calls connect/enable/homing on hardware.
"""
import argparse,json,os,socket,subprocess,tempfile,time,urllib.request,urllib.error
from pathlib import Path
ap=argparse.ArgumentParser()
ap.add_argument('--project',required=True)
ap.add_argument('--python',required=True)
ap.add_argument('--out',required=True)
args=ap.parse_args()
root=Path(args.project)
trace={}
with tempfile.TemporaryDirectory(prefix='hand-http-contract-') as temp:
    temp=Path(temp)
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0)); port=sock.getsockname()[1]
    config=temp/'config.json'
    config.write_text(json.dumps({'host':'127.0.0.1','port':port,'data_dir':str(temp/'motions')}))
    base=f'http://127.0.0.1:{port}'
    headers={'X-Motion-Client':'differential-contract-20261006','Origin':base,'Content-Type':'application/json'}
    def req(action,body=None):
        request=urllib.request.Request(base+'/api/'+action,data=None if body is None else json.dumps(body).encode(),headers=headers)
        try:
            with urllib.request.urlopen(request,timeout=15) as response: return json.load(response)
        except urllib.error.HTTPError as e: raise RuntimeError(e.read().decode()) from e
    log=(temp/'server.log').open('w')
    process=subprocess.Popen([args.python,str(root/'online_control/online_motion_play/server.py'),'--config',str(config),'--no-browser'],cwd=root,stdout=log,stderr=log,env={**os.environ,'QT_QPA_PLATFORM':'offscreen','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})
    def wait_for(fn,timeout=15):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if process.poll() is not None:
                log.flush(); raise RuntimeError((temp/'server.log').read_text())
            try:
                value=fn()
                if value: return value
            except (OSError,urllib.error.URLError): pass
            time.sleep(.02)
        raise RuntimeError('Timed out waiting for HTTP state')
    def state(): return req('state')
    def clean(value):
        if isinstance(value,float): return round(value,3)
        if isinstance(value,list): return [clean(x) for x in value]
        if isinstance(value,dict): return {k:clean(v) for k,v in value.items() if k not in ('id','tip','text','title','playback_timing','control','sdk_local_access')}
        return value
    def record(label):
        s=state()
        trace[label]=clean({k:s[k] for k in ('model_id','side','pose','count','connected','playing','homing','controls','joints','dialogs') if k in s})
        trace[label+'_export']=clean(req('export'))
    def widget(key,value=None,joint=False):
        s=state()
        spec=next(x for x in s['joints'] if x['short']==key) if joint else s['controls'][key]
        body={'id':spec['id']}
        if value is not None: body['value']=value
        req('widget',body)
        time.sleep(.05)
    def dialog_button(prefer=('OK','Yes','确认','确定')):
        dialogs=wait_for(lambda:state()['dialogs'])
        d=next((x for x in dialogs if x.get('modal')),dialogs[0])
        buttons=[x for x in d['items'] if x['kind']=='button']
        b=next((b for label in prefer for b in buttons if b.get('text','').replace('&','')==label),None)
        if b is None: raise RuntimeError(f'Missing confirm button: {buttons}')
        req('widget',{'id':b['id']})
        time.sleep(.06)
    def dismiss():
        for _ in range(4):
            if not state()['dialogs']: return
            dialog_button(('OK','确定','确认','Close','关闭','Cancel','取消','No'))
        raise RuntimeError('Dialogs did not close')
    try:
        wait_for(lambda:req('health'))
        req('control',{})
        record('initial')
        widget('thumb_swing',.6,True); widget('index_mcp',.2,True)
        widget('transition',.25);widget('interval',.015)
        record('edited')
        widget('savemotion')
        d=wait_for(lambda:state()['dialogs'])[0]
        name=next(x for x in d['items'] if x['kind']=='text')
        req('widget',{'id':name['id'],'value':'动作甲'})
        dialog_button();dismiss()
        record('saved')
        widget('thumb_swing',1.2,True);widget('savemotion')
        d=wait_for(lambda:state()['dialogs'])[0]
        req('widget',{'id':next(x for x in d['items'] if x['kind']=='text')['id'],'value':'动作乙'})
        dialog_button();dismiss()
        record('inserted')
        widget('choosemotion',0);widget('loadmotion');time.sleep(.1)
        record('loaded')
        widget('deletemotion');dialog_button();dismiss()
        record('deleted')
        data={'_hand_side':'right','姿态一':{'left_hand_thumb_swing_joint':-.4,'left_hand_pinky_rota_joint':-.3,'left_hand_index_swing_joint':.2,'_transition_s':.012,'_interval_s':0}}
        req('import',{'text':'\ufeff'+json.dumps(data)})
        record('right_imported')
        widget('loadmotion');time.sleep(.1)
        record('right_loaded')
        req('import',{'text':json.dumps([{'name':'旧动作','left_hand_index_MCP_joint':.6}])})
        record('legacy_imported')
        before=req('export')
        try: req('import',{'text':'{"bad":{"left_hand_index_MCP_joint":"oops"}}'})
        except RuntimeError: pass
        else: raise AssertionError('invalid import accepted')
        assert req('export')==before
        for model in ('TsenseI_v3_0','TsenseI_v3_0_withid','TsenseI_v3_2','TsenseI_v3_2_withid','TsenseI_v2'):
            req('model',{'model_id':model});record(model)
        req('release',{})
        trace['released_available']=state()['control']['available']
        Path(args.out).write_text(json.dumps(trace,ensure_ascii=False,indent=2)+'\n')
        print(f'HTTP contract captured: {len(trace)} entries -> {args.out}')
    finally:
        process.terminate()
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.kill();process.wait()
        log.close()
