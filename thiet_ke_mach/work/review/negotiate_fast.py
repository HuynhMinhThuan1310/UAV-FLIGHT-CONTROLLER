import pathlib
src=pathlib.Path('work/review/route_single.py').read_text(encoding='utf-8-sig')
src=src.replace('STEP=.2','STEP=.1')
src=src.replace("'J23':(87,73,90)","'J23':(84,73,90)")
exec(src[:src.index('best=999;')])
# Negotiated-congestion routing. Pads remain hard obstacles; traces are rerouted until separate.
import ctypes
lib=ctypes.CDLL(str(pathlib.Path('work/review/pathfinder.dll').resolve()))
lib.findpath.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p]
lib.findpath.restype=ctypes.c_int

def search(block,costs,start,targets):
 block=np.ascontiguousarray(block,dtype=np.uint8); costs=np.ascontiguousarray(costs,dtype=np.float32); tg=np.array(list(targets),dtype=np.int32);out=np.empty(NX*NY,dtype=np.int32)
 count=lib.findpath(NX,NY,block.ctypes.data,costs.ctypes.data,start,tg.ctypes.data,len(tg),out.ctypes.data)
 return out[:count].tolist() if count else None

paths={};cells={};history=np.zeros((NY,NX),dtype=np.float32);best=999;start=time.time();random.seed(991)
for iteration in range(80):
 order=nets.copy();random.shuffle(order)
 if iteration==0:order.sort(key=lambda n:len(netpads[n]))
 for n in order:
  costs=history.copy()
  for other,cc in cells.items():
   if other==n:continue
   mask=np.zeros((NY,NX),dtype=np.bool_);stamp(mask,cc,(width[n]+width[other])/2+.20+.045)
   costs[mask]+=3+iteration*2
  tree={netpads[n][0][0]};pending=netpads[n][1:].copy();branches=[]
  while pending:
   ix=min(range(len(pending)),key=lambda i:min(abs(pending[i][0]%NX-c%NX)+abs(pending[i][0]//NX-c//NX) for c in tree))
   p=pending.pop(ix);path=search(base[n],costs,p[0],tree)
   if path is not None:tree.update(path);branches.append(path)
  paths[n]=branches;cells[n]=tree
 conflicts=[];bad=set()
 for i,n in enumerate(nets):
  for m in nets[i+1:]:
   mask=np.zeros((NY,NX),dtype=np.bool_);stamp(mask,cells[m],(width[n]+width[m])/2+.20+.045)
   hits=[c for c in cells[n] if mask[c//NX,c%NX]]
   if hits:
    conflicts.append((n,m,len(hits)));bad.update([n,m])
    for c in hits:history[c//NX,c%NX]+=1.5
 print('ITER',iteration,'conflict pairs',len(conflicts),'time',round(time.time()-start),flush=True)
 # Save only a collision-free subset. Prefer removing minimum routing branches, then reroute strictly.
 if len(conflicts)<best:
  best=len(conflicts)
  (W/'negotiated.json').write_text(json.dumps({'iteration':iteration,'conflicts':[(names[n],names[m],c) for n,m,c in conflicts]}))
  with (W/'paths.json').open('w') as out:json.dump({str(n):p for n,p in paths.items()},out)
 if not conflicts:
  save(paths,[],iteration);break

