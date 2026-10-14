import pcbnew as k
from PIL import Image,ImageDraw,ImageFont

b=k.LoadBoard('mach_uav.kicad_pcb');scale=14;x0=y0=50
im=Image.new('RGB',(1390,1390),'#111a22');d=ImageDraw.Draw(im)
font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',14)
tiny=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',11)
xy=lambda p:((k.ToMM(p.x)-x0)*scale,(k.ToMM(p.y)-y0)*scale)
for t in b.GetTracks():
 d.line((xy(t.GetStart()),xy(t.GetEnd())),fill='#4ba9ed' if k.ToMM(t.GetWidth())>=.762 else '#3979b1',width=max(2,round(k.ToMM(t.GetWidth())*scale)),joint='curve')
for s in b.GetDrawings():
 if s.GetLayer()==k.Edge_Cuts:d.line((xy(s.GetStart()),xy(s.GetEnd())),fill='white',width=2)
for f in b.GetFootprints():
 for p in f.Pads():
  x,y=xy(p.GetPosition());r=min(k.ToMM(p.GetSize().x),k.ToMM(p.GetSize().y))*scale/2
  col='#90e6aa' if f.GetReference() in ('W5B','W6B') else '#aaadb0' if p.GetNetname()=='G' else '#e4b053'
  d.ellipse((x-r,y-r,x+r,y+r),fill=col)
  if p.GetDrillSize().x:
   rr=k.ToMM(p.GetDrillSize().x)*scale/2
   d.ellipse((x-rr,y-rr,x+rr,y+rr),fill='#111a22')
 if f.GetReference()!='W1':
  x,y=xy(f.GetPosition());d.text((x-10,y-30),f.GetReference(),font=font,fill='#ffffff')
for s in b.GetDrawings():
 if isinstance(s,k.PCB_TEXT) and s.GetLayer()==k.F_SilkS:
  x,y=xy(s.GetPosition())
  if s.GetText().startswith('W') or s.GetText().startswith('G:'):
   d.text((x,y-7),s.GetText(),font=tiny,fill='#e7f2f8')
  elif s.GetText() in ('1','2','3','4','5','6','7','8'):
   d.text((x-4,y-6),s.GetText(),font=tiny,fill='white')
d.text((52,20),'B.Cu: 3.3 V da noi • 0 via • 0 GND trace • W1-W6 can han ngoai',font=font,fill='#d7ebf4')
im.save('work/prior_layout_adapt/pcb-preview.png')
