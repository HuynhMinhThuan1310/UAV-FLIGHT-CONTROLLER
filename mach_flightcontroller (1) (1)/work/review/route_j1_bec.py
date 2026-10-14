import pathlib, pcbnew as k, math
s=pathlib.Path('work/review/route_changed_nets.py').read_text()
s=s.replace("base=k.LoadBoard(str(w/'mach_uav.kicad_pcb'))","base=k.LoadBoard(str(w/'routed-plus-bec.kicad_pcb'))")
s=s.replace("netnames=['/5V_BEC','/5V_BEC1','/USART1_RX','/USART1_TX','/USART3_RX','/USART3_TX']","netnames=['/5V_BEC']")
exec(s[:s.index('random.seed(91)')])
n='/5V_BEC';p=next(p for p in netpads[n] if p[3]=='J1' and p[4]=='2')
same=[t for t in base.GetTracks() if t.GetNetname()==n]
targetpts={}
for t in same:
 for pt in (t.GetStart(),t.GetEnd()):
  x,y=k.ToMM(pt.x),k.ToMM(pt.y)
  targetpts[cell(x,y)]=(x,y)
for width in (.889,.762,.5,.35):
 path=search(baseblock(n,width),p[0],set(targetpts))
 print('width',width,'path',len(path) if path else None,flush=True)
 if path:
  coords=[(p[1],p[2])]+[xy(c) for c in path]+[targetpts[path[-1]]]
  b=k.LoadBoard(str(w/'routed-plus-bec.kicad_pcb'))
  simple=[coords[0]]
  for i in range(1,len(coords)-1):
   a=simple[-1];c=coords[i];z=coords[i+1]
   if abs((c[0]-a[0])*(z[1]-c[1])-(c[1]-a[1])*(z[0]-c[0]))>1e-7:simple.append(c)
  simple.append(coords[-1])
  for a,z in zip(simple,simple[1:]):
   if math.dist(a,z)<1e-6:continue
   t=k.PCB_TRACK(b);t.SetStart(k.VECTOR2I(k.FromMM(a[0]),k.FromMM(a[1])));t.SetEnd(k.VECTOR2I(k.FromMM(z[0]),k.FromMM(z[1])));t.SetWidth(k.FromMM(width));t.SetLayer(k.B_Cu);t.SetNetCode(netcodes[n]);b.Add(t)
  k.SaveBoard(str(w/'routed-plus-bec-j1.kicad_pcb'),b)
  break
