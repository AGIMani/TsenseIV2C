import json
from pathlib import Path
a=json.loads(Path('/tmp/hand-ubuntu-20261006/original-http-contract.json').read_text())
b=json.loads(Path('/tmp/hand-ubuntu-20261006/qtless-http-contract.json').read_text())
diffs=[]
def diff(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        for k in a.keys()|b.keys():
            if k not in a or k not in b: diffs.append((path+'/'+k,a.get(k),b.get(k)))
            else: diff(a[k],b[k],path+'/'+k)
    elif isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
        for i,(x,y) in enumerate(zip(a,b)):diff(x,y,path+f'/{i}')
    elif a!=b: diffs.append((path,a,b))
diff(a,b)
kinds={}
for path,x,y in diffs:
    key='enabled' if path.endswith('/enabled') else 'other'
    kinds[key]=kinds.get(key,0)+1
    if key=='other': print(json.dumps({'path':path,'old':x,'new':y},ensure_ascii=False))
print(json.dumps({'differences':len(diffs),'kinds':kinds}))
Path('/tmp/hand-ubuntu-20261006/http-contract-diff.json').write_text(json.dumps(diffs,ensure_ascii=False,indent=2))
