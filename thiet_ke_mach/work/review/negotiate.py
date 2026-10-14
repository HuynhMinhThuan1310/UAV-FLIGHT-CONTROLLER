import pathlib
src=pathlib.Path('work/review/route_single.py').read_text(encoding='utf-8-sig')
src=src.replace("'J23':(87,73,90)","'J23':(84,73,90)")
exec(src[:src.index('best=999;')])
# Negotiated-congestion routing. Pads remain hard obstacles; traces are rerouted until separate.
def search(block,costs,start,targets):
 flat=block.ravel(); costs=costs.ravel();targets=set(targets)
 tx=[p%NX for p in targets];ty=[p//NX for p in targets];xmin,xmax=min(tx),max(tx);ymin,ymax=min(ty),max(ty)
 def h(x,y):
  dx=max(xmin-x,0,x-xmax);dy=max(ymin-y,0,y-ymax);return max(dx,dy)+.41421356*min(dx,dy)
 dist={start:0.};prev={};heap=[(h(start%NX,start//NX),0.,start)];seen=set()
 while heap:
  _,cost,c=heapq.heappop(heap)
  if c in seen:continue
  if c in targets:
   path=[c]
   while c!=start:c=prev[c];path.append(c)
   return path[::-1]
  seen.add(c);x=c%NX;y=c//NX
  for dx,dy,dd in moves:
   xx=x+dx;yy=y+dy
   if xx<0 or xx>=NX or yy<0 or yy>=NY:continue
   v=yy*NX+xx
   if flat[v] and v not in targets:continue
   if dx and dy and (flat[y*NX+xx] or flat[yy*NX+x]):continue
   nc=cost+dd*(1+float(costs[v]))
   if nc<dist.get(v,1e20):
    dist[v]=nc;prev[v]=c;heapq.heappush(heap,(nc+h(xx,yy)*1.08,nc,v))
 return None
paths={};cells={};history=np.zeros((NY,NX),dtype=np.float32);best=999;start=time.time();random.seed(991)
for iteration in range(35):
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
