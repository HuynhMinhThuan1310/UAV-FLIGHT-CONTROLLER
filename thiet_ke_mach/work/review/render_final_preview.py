import pcbnew as k
from PIL import Image, ImageDraw, ImageFont

b=k.LoadBoard('mach_uav.kicad_pcb')
scale=16
x0,y0=59,56
im=Image.new('RGB',(1320,1460),'#111b23')
d=ImageDraw.Draw(im)
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',16)
small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',11)
xy=lambda p:((k.ToMM(p.x)-x0)*scale,(k.ToMM(p.y)-y0)*scale)

for t in b.GetTracks():
 col='#3db9ee' if k.ToMM(t.GetWidth())>.5 else '#357fb4'
 d.line((xy(t.GetStart()),xy(t.GetEnd())),fill=col,width=max(3,round(k.ToMM(t.GetWidth())*scale)),joint='curve')
for s in b.GetDrawings():
 if s.GetLayer()==k.Edge_Cuts:
  d.line((xy(s.GetStart()),xy(s.GetEnd())),fill='#ffffff',width=3)

fps={f.GetReference():f for f in b.GetFootprints()}
u=fps['U3']; bb=u.GetBoundingBox()
left=(k.ToMM(bb.GetX())-x0)*scale; top=(k.ToMM(bb.GetY())-y0)*scale
right=(k.ToMM(bb.GetRight())-x0)*scale; bottom=(k.ToMM(bb.GetBottom())-y0)*scale
d.rounded_rectangle((left+7,top+13,right-7,bottom-13),radius=10,outline='#708596',width=2)

for f in b.GetFootprints():
 for p in f.Pads():
  x,y=xy(p.GetPosition()); sx,sy=k.ToMM(p.GetSize());r=min(sx,sy)*scale/2
  color='#8df0b3' if f.GetReference()=='W3B' else '#e7ae59' if p.GetNetname()!='G' else '#a9a9a9'
  d.ellipse((x-r,y-r,x+r,y+r),fill=color)
  if p.GetDrillSize().x:
   rd=k.ToMM(p.GetDrillSize().x)*scale/2
   d.ellipse((x-rd,y-rd,x+rd,y+rd),fill='#111b23')
  if f.GetReference()=='J32': d.text((x+8,y-9),p.GetNumber(),font=small,fill='#ffffff')

for f in b.GetFootprints():
 ref=f.GetReference();x,y=xy(f.GetPosition())
 if ref=='U3':d.text((x-14,y-12),'U3 / BLUEPILL',font=font,fill='#ffffff')
 elif ref=='J26':d.text((x-25,y-70),'J26 GPS (B)',font=font,fill='#ffffff')
 elif ref=='W3B':d.text((x+10,y-15),'W3B',font=small,fill='#8df0b3')
 elif ref=='J32':d.text((x-15,y-65),'J32 NRF',font=font,fill='#ffffff')
 else:d.text((x-12,y-33),ref,font=font,fill='#ffffff')

d.text((60,35),'B.Cu only  •  no GND traces  •  external wires W1–W8',font=font,fill='#dbeaf3')
im.save('work/flightcontroller/pcb-preview.png')
