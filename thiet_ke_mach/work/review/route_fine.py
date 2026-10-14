import pcbnew as k, numpy as np, math, heapq, random, time, json, pathlib, shutil
ROOT=pathlib.Path.cwd(); W=ROOT/'work'/'single_layer'; W.mkdir(parents=True,exist_ok=True)
SRC=ROOT/'mach_uav.kicad_pcb'
if not (W/'original.kicad_pcb').exists(): shutil.copy2(SRC,W/'original.kicad_pcb')
b=k.LoadBoard(str(SRC))
removed=list(b.GetTracks())+list(b.GetDrawings())+list(b.Zones())
for t in removed: b.Remove(t)


fps={f.GetReference():f for f in b.GetFootprints()}
pos={'U3':(100,100,0),'J8':(80,86,90),'J15':(80,89.5,90),'J16':(80,93,90),'J17':(80,96.5,90),'J2':(81,102,0),'J18':(81,117,180),'J32':(119,98,0),'J20':(119,88,0),'J21':(127,92,180),'J26':(121,117,0),'J24':(118,126,90),'J25':(127,125,90),'J19':(81,123,90),'J23':(84,73,90),'J27':(115,73,90),'J1':(123,73,90)}
for r,f in fps.items():
 if r=='J26' and f.GetLayer()!=k.B_Cu: f.Flip(f.GetPosition(),k.FLIP_DIRECTION_LEFT_RIGHT)
 x,y,a=pos[r]; f.SetOrientationDegrees(a); f.SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y)))
 f.Reference().SetTextSize(k.VECTOR2I(k.FromMM(.9),k.FromMM(.9))); f.Reference().SetTextThickness(k.FromMM(.15)); f.Value().SetVisible(False)
 f.Reference().SetTextAngle(k.EDA_ANGLE(0,k.DEGREES_T))
 f.Reference().SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y-2.5)))
# Closed 58 x 64 mm outline
X0,Y0,X1,Y1=74,68,132,132
for p,q in [((X0,Y0),(X1,Y0)),((X1,Y0),(X1,Y1)),((X1,Y1),(X0,Y1)),((X0,Y1),(X0,Y0))]:
 d=k.PCB_SHAPE(); d.SetShape(k.SHAPE_T_SEGMENT); d.SetLayer(k.Edge_Cuts); d.SetWidth(k.FromMM(.05)); d.SetStart(k.VECTOR2I(k.FromMM(p[0]),k.FromMM(p[1]))); d.SetEnd(k.VECTOR2I(k.FromMM(q[0]),k.FromMM(q[1]))); b.Add(d)
k.SaveBoard(str(W/'placement.kicad_pcb'),b)
STEP=.1; NX=round((X1-X0)/STEP)+1; NY=round((Y1-Y0)/STEP)+1; N=NX*NY
coords=lambda x,y:(round((x-X0)/STEP),round((y-Y0)/STEP))
xy=lambda c:(X0+c%NX*STEP,Y0+c//NX*STEP)
netpads={}; pads=[]
for f in b.GetFootprints():
 for p in f.Pads():
  x,y=k.ToMM(p.GetPosition()); sx,sy=k.ToMM(p.GetSize()); a=math.radians(p.GetOrientationDegrees()); net=p.GetNetCode(); cx,cy=coords(x,y)
  pads.append((x,y,sx,sy,a,net,int(p.GetShape()),f.GetReference(),p.GetNumber()))
  if net>0: netpads.setdefault(net,[]).append((cy*NX+cx,x,y,f.GetReference(),p.GetNumber()))
nets=[n for n,p in netpads.items() if len(p)>1]; names={n:b.FindNet(n).GetNetname() for n in nets}
# Existing clearance .20 mm, signal .25 mm, power .45 mm.
width={n:(.45 if names[n] in ['G','3.3'] or '5V' in names[n] else .25) for n in nets}
base={}
for n in nets:
 arr=np.zeros((NY,NX),dtype=np.bool_); arr[:4,:]=True;arr[-4:,:]=True;arr[:,:4]=True;arr[:,-4:]=True
 margin=.20+width[n]/2+.025
 for x,y,sx,sy,a,nn,shape,ref,pnum in pads:
  if nn==n: continue
  rad=max(sx,sy)/2+margin+.2; cx,cy=coords(x,y); rg=math.ceil(rad/STEP)+1
  xa,xb=max(0,cx-rg),min(NX,cx+rg+1);ya,yb=max(0,cy-rg),min(NY,cy+rg+1)
  xx,yy=np.meshgrid(X0+np.arange(xa,xb)*STEP-x,Y0+np.arange(ya,yb)*STEP-y)
  u=xx*math.cos(a)+yy*math.sin(a);v=-xx*math.sin(a)+yy*math.cos(a)
  if shape==int(k.PAD_SHAPE_CIRCLE): mask=xx*xx+yy*yy<=(sx/2+margin)**2
  else: mask=np.maximum(abs(u)-sx/2,0)**2+np.maximum(abs(v)-sy/2,0)**2<=margin**2
  arr[ya:yb,xa:xb]|=mask
 base[n]=arr
moves=[(1,0,1.),(-1,0,1.),(0,1,1.),(0,-1,1.),(1,1,1.41421356),(1,-1,1.41421356),(-1,1,1.41421356),(-1,-1,1.41421356)]
def astar(block,start,targets):
 flat=block.ravel(); targets=set(targets); tx=np.array([p%NX for p in targets]);ty=np.array([p//NX for p in targets]); xmin,xmax=tx.min(),tx.max();ymin,ymax=ty.min(),ty.max()
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
   nc=cost+dd
   if nc<dist.get(v,1e20):
    dist[v]=nc;prev[v]=c;heapq.heappush(heap,(nc+h(xx,yy),nc,v))
 return None
def stamp(arr,cells,radius):
 rr=math.ceil(radius/STEP);cells=np.array(list(cells));xx=cells%NX;yy=cells//NX
 for dx in range(-rr,rr+1):
  for dy in range(-rr,rr+1):
   if (dx*STEP)**2+(dy*STEP)**2>radius*radius:continue
   x=xx+dx;y=yy+dy;valid=(x>=0)&(x<NX)&(y>=0)&(y<NY);arr[y[valid],x[valid]]=True

def route(order):
 done={};failed=[];paths={}
 for n in order:
  obstacle=base[n].copy()
  for other,cells in done.items():stamp(obstacle,cells,(width[n]+width[other])/2+.20+.04)
  pp=netpads[n]; tree={pp[0][0]}; branches=[];pending=pp[1:].copy()
  while pending:
   # join closest remaining endpoint to routed tree
   ix=min(range(len(pending)),key=lambda i:min(abs(pending[i][0]%NX-c%NX)+abs(pending[i][0]//NX-c//NX) for c in tree))
   p=pending.pop(ix);path=astar(obstacle,p[0],tree)
   if path is None:failed.append((n,p[3],p[4]));continue
   tree.update(path); branches.append(path)
  if branches: done[n]=tree;paths[n]=branches
 return paths,failed

def save(paths,failed,attempt):
 for t in list(b.GetTracks()):b.Remove(t)
 def track(n,a,z):
  if math.dist(a,z)<.00001:return
  t=k.PCB_TRACK(b);t.SetStart(k.VECTOR2I(k.FromMM(a[0]),k.FromMM(a[1])));t.SetEnd(k.VECTOR2I(k.FromMM(z[0]),k.FromMM(z[1])));t.SetWidth(k.FromMM(width[n]));t.SetLayer(k.B_Cu);t.SetNetCode(n);b.Add(t)
 for n,branches in paths.items():
  for path in branches:
   simple=[path[0]]
   for i in range(1,len(path)-1):
    if path[i]-path[i-1]!=path[i+1]-path[i]:simple.append(path[i])
   simple.append(path[-1])
   for a,z in zip(simple,simple[1:]):track(n,xy(a),xy(z))
  used=set(c for path in branches for c in path)
  for c,x,y,ref,num in netpads[n]:
   if c in used:track(n,(x,y),xy(c))
 k.SaveBoard(str(W/'candidate.kicad_pcb'),b)
 (W/'routing.json').write_text(json.dumps({'attempt':attempt,'failed':[(names[n],r,p) for n,r,p in failed],'tracks':len(list(b.GetTracks())),'size_mm':[X1-X0,Y1-Y0]},indent=2))
 print('SAVED',attempt,'unrouted branches',len(failed),[(names[n],r,p) for n,r,p in failed],flush=True)

best=999; start=time.time();random.seed(713)
span=lambda n:max(p[1] for p in netpads[n])-min(p[1] for p in netpads[n])+max(p[2] for p in netpads[n])-min(p[2] for p in netpads[n])
for attempt in range(4):
 if attempt==0:order=sorted(nets,key=lambda n:(len(netpads[n])>3,span(n)))
 else:order=sorted(nets,key=lambda n:span(n)*random.uniform(.3,2.2))
 paths,failed=route(order)
 print('ATTEMPT',attempt,'failed',len(failed),'seconds',round(time.time()-start),flush=True)
 if len(failed)<best:best=len(failed);save(paths,failed,attempt)
 if best==0:break


