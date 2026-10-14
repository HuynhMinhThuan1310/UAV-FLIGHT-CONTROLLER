import pcbnew as k,math
b=k.LoadBoard('work/prior_layout_adapt/mach_uav.kicad_pcb')
f=next(f for f in b.GetFootprints() if f.GetReference()=='J18')
otherpads=[p for g in b.GetFootprints() if g.GetReference()!='J18' for p in g.Pads()]
tracks=list(b.GetTracks())
def dseg(x,y,a,c):
 ax,ay=k.ToMM(a.x),k.ToMM(a.y);bx,by=k.ToMM(c.x),k.ToMM(c.y)
 dx=bx-ax;dy=by-ay;u=max(0,min(1,((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy+1e-20)))
 return math.hypot(x-ax-u*dx,y-ay-u*dy)
results=[]
for angle in (0,180):
 for x in (138,139,140,141,142):
  for y in range(91,125):
   f.SetOrientationDegrees(angle);f.SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y)))
   pads=list(f.Pads());pos=[(k.ToMM(p.GetPosition().x),k.ToMM(p.GetPosition().y),p.GetNetname()) for p in pads]
   if any(px<55 or px>145 or py<55 or py>145 for px,py,n in pos):continue
   conflicts=0;nearest=99
   for px,py,n in pos:
    for t in tracks:
     if t.GetNetname()==n:continue
     dist=dseg(px,py,t.GetStart(),t.GetEnd())-.4-k.ToMM(t.GetWidth())/2
     nearest=min(nearest,dist)
     if dist<.22:conflicts+=1
    for q in otherpads:
     if q.GetNetname()==n:continue
     qx,qy=k.ToMM(q.GetPosition().x),k.ToMM(q.GetPosition().y)
     dist=math.hypot(px-qx,py-qy)-.4-max(k.ToMM(q.GetSize().x),k.ToMM(q.GetSize().y))/2
     if dist<.2:conflicts+=1
   # favor locations close to U3 right side pins 26-27 near (107.62,110)
   score=conflicts*100+abs(y-106)*.1+abs(x-141)*.1
   results.append((score,conflicts,round(nearest,2),x,y,angle))
print(*sorted(results)[:30],sep='\n')
