import pcbnew as k
from PIL import Image,ImageDraw,ImageFont
b=k.LoadBoard('work/single_layer/candidate.kicad_pcb');sc=15;x0=72;y0=66
im=Image.new('RGB',(930,1050),'#101820');d=ImageDraw.Draw(im)
pix=lambda p:((k.ToMM(p.x)-x0)*sc,(k.ToMM(p.y)-y0)*sc)
for t in b.GetTracks():d.line([pix(t.GetStart()),pix(t.GetEnd())],fill='#58b4ef',width=max(1,round(k.ToMM(t.GetWidth())*sc)))
for f in b.GetFootprints():
 for p in f.Pads():
  x,y=pix(p.GetPosition());sx,sy=k.ToMM(p.GetSize());d.ellipse((x-sx*sc/2,y-sy*sc/2,x+sx*sc/2,y+sy*sc/2),fill='#e4b852');d.text((x+5,y-10),p.GetNumber(),fill='white')
 x,y=pix(f.GetPosition());d.text((x-10,y-28),f.GetReference()+' '+f.GetValue(),fill='#ffffff')
for shape in b.GetDrawings():
 if shape.GetLayer()==k.Edge_Cuts:d.line([pix(shape.GetStart()),pix(shape.GetEnd())],fill='white',width=2)
im.save('work/single_layer/preview.png')
