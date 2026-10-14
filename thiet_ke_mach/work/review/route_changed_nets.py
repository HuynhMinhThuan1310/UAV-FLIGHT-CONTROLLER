import pcbnew as k, numpy as np, math, random, pathlib, ctypes, json, time, os

os.dup2(os.open(os.devnull,os.O_WRONLY),2)
root=pathlib.Path.cwd();w=root/'work/prior_layout_adapt'
base=k.LoadBoard(str(w/'mach_uav.kicad_pcb'))
netnames=['/5V_BEC','/5V_BEC1','/USART1_RX','/USART1_TX','/USART3_RX','/USART3_TX']
netcodes={name:base.FindNet(name).GetNetCode() for name in netnames}
STEP=.2;X0=54;Y0=54;X1=146;Y1=146;NX=round((X1-X0)/STEP)+1;NY=round((Y1-Y0)/STEP)+1
xy=lambda c:(X0+(c%NX)*STEP,Y0+(c//NX)*STEP)
cell=lambda x,y:round((y-Y0)/STEP)*NX+round((x-X0)/STEP)
padinfo=[];netpads={n:[] for n in netnames}
for f in base.GetFootprints():
 for p in f.Pads():
  x,y=k.ToMM(p.GetPosition());sx,sy=k.ToMM(p.GetSize());n=p.GetNetname()
  padinfo.append((x,y,sx,sy,math.radians(p.GetOrientationDegrees()),p.GetShape(),n))
  if n in netpads:netpads[n].append((cell(x,y),x,y,f.GetReference(),p.GetNumber()))
trackinfo=[]
for t in base.GetTracks():
 a=t.GetStart();z=t.GetEnd()
 trackinfo.append((k.ToMM(a.x),k.ToMM(a.y),k.ToMM(z.x),k.ToMM(z.y),k.ToMM(t.GetWidth()),t.GetNetname()))

lib=ctypes.CDLL(str((root/'work/review/pathfinder.dll').resolve()))
lib.findpath.argtypes=[ctypes.c_int,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p]
lib.findpath.restype=ctypes.c_int
def search(block,start,targets):
 tg=np.array(list(targets),dtype=np.int32);out=np.empty(NX*NY,dtype=np.int32)
 costs=np.zeros((NY,NX),dtype=np.float32)
 arr=np.ascontiguousarray(block,dtype=np.uint8)
 count=lib.findpath(NX,NY,arr.ctypes.data,costs.ctypes.data,start,tg.ctypes.data,len(tg),out.ctypes.data)
 return out[:count].tolist() if count else None

def stamp_seg(arr,ax,ay,bx,by,r):
 xa=max(0,int(math.floor((min(ax,bx)-r-X0)/STEP)));xb=min(NX,int(math.ceil((max(ax,bx)+r-X0)/STEP))+1)
 ya=max(0,int(math.floor((min(ay,by)-r-Y0)/STEP)));yb=min(NY,int(math.ceil((max(ay,by)+r-Y0)/STEP))+1)
 if xa>=xb or ya>=yb:return
 xs=X0+np.arange(xa,xb)*STEP;ys=Y0+np.arange(ya,yb)*STEP
 xx,yy=np.meshgrid(xs,ys);dx=bx-ax;dy=by-ay;den=dx*dx+dy*dy
 if den<1e-9:dist=(xx-ax)**2+(yy-ay)**2
 else:
  u=np.clip(((xx-ax)*dx+(yy-ay)*dy)/den,0,1)
  dist=(xx-ax-u*dx)**2+(yy-ay-u*dy)**2
 arr[ya:yb,xa:xb]|=dist<=r*r

cache={}
def baseblock(net,width):
 key=(net,width)
 if key in cache:return cache[key]
 arr=np.zeros((NY,NX),dtype=np.bool_)
 border=math.ceil((.6+width/2)/STEP)
 arr[:border,:]=True;arr[-border:,:]=True;arr[:,:border]=True;arr[:,-border:]=True
 margin=.2+width/2+.06
 for x,y,sx,sy,angle,shape,pn in padinfo:
  if pn==net:continue
  rad=max(sx,sy)/2+margin+.25
  cx=round((x-X0)/STEP);cy=round((y-Y0)/STEP);rg=math.ceil(rad/STEP)
  xa=max(0,cx-rg);xb=min(NX,cx+rg+1);ya=max(0,cy-rg);yb=min(NY,cy+rg+1)
  xx,yy=np.meshgrid(X0+np.arange(xa,xb)*STEP-x,Y0+np.arange(ya,yb)*STEP-y)
  u=xx*math.cos(angle)+yy*math.sin(angle);v=-xx*math.sin(angle)+yy*math.cos(angle)
  if int(shape)==int(k.PAD_SHAPE_CIRCLE):mask=xx*xx+yy*yy<=(sx/2+margin)**2
  else:mask=np.maximum(abs(u)-sx/2,0)**2+np.maximum(abs(v)-sy/2,0)**2<=margin**2
  arr[ya:yb,xa:xb]|=mask
 for ax,ay,bx,by,tw,tn in trackinfo:
  if tn!=net:stamp_seg(arr,ax,ay,bx,by,(tw+width)/2+.2+.06)
 cache[key]=arr
 return arr

def route(order):
 segments=[];failed=[]
 for n in order:
  pp=netpads[n];tree={pp[0][0]};pending=pp[1:].copy()
  while pending:
   p=pending.pop(0)
   result=None
   for width in ((.889,.762,.5) if '5V' in n else (.889,.762,.5,.35)):
    block=baseblock(n,width).copy()
    for on,ow,coords in segments:
     if on==n:continue
     for (ax,ay),(bx,by) in zip(coords,coords[1:]):stamp_seg(block,ax,ay,bx,by,(width+ow)/2+.2+.06)
    path=search(block,p[0],tree)
    if path:
     coords=[(p[1],p[2])]+[xy(c) for c in path]
     if path[-1]==pp[0][0]:coords.append((pp[0][1],pp[0][2]))
     result=(width,path,coords);break
   if result is None:failed.append((n,p[3],p[4]));continue
   width,path,coords=result;tree.update(path)
   segments.append((n,width,coords))
 return segments,failed

def save(segments,failed,attempt):
 b=k.LoadBoard(str(w/'mach_uav.kicad_pcb'))
 for n,width,coords in segments:
  simple=[coords[0]]
  for i in range(1,len(coords)-1):
   a=simple[-1];c=coords[i];z=coords[i+1]
   if abs((c[0]-a[0])*(z[1]-c[1])-(c[1]-a[1])*(z[0]-c[0]))>1e-7:simple.append(c)
  simple.append(coords[-1])
  for a,z in zip(simple,simple[1:]):
   if math.dist(a,z)<1e-6:continue
   t=k.PCB_TRACK(b);t.SetStart(k.VECTOR2I(k.FromMM(a[0]),k.FromMM(a[1])));t.SetEnd(k.VECTOR2I(k.FromMM(z[0]),k.FromMM(z[1])));t.SetWidth(k.FromMM(width));t.SetLayer(k.B_Cu);t.SetNetCode(netcodes[n]);b.Add(t)
 k.SaveBoard(str(w/'routed.kicad_pcb'),b)
 (w/'routing.json').write_text(json.dumps({'attempt':attempt,'failed':failed,'new_tracks':len(list(b.GetTracks()))-len(trackinfo),'segments':[(n,wd,len(coords)) for n,wd,coords in segments]},indent=2))

random.seed(91);best=(999,-1);start=time.time()
for attempt in range(80):
 order=netnames.copy();random.shuffle(order)
 if attempt%3==0:order.sort(key=lambda n:('5V' not in n,n))
 segments,failed=route(order)
 quality=(len(failed),-sum(len(coords) for n,w,coords in segments if w>=.762))
 if quality<best:
  best=quality;save(segments,failed,attempt)
  print('best',attempt,'failed',len(failed),failed,'seconds',round(time.time()-start),flush=True)
 if not failed:break
