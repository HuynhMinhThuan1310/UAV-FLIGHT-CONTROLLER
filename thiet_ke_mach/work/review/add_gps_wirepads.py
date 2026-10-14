import pcbnew as k, pathlib,os
os.dup2(os.open(os.devnull,os.O_WRONLY),2)
root=pathlib.Path.cwd();w=root/'work/prior_layout_adapt'
b=k.LoadBoard(str(w/'routed-plus-bec.kicad_pcb'))
fps={f.GetReference():f for f in b.GetFootprints()};gps=fps['J26']
entries=[
 ('W5B','2',132.0,138.0,[(136.18,131.5),(132.0,131.5),(132.0,138.0)]),
 ('W6B','1',136.18,138.0,[(136.18,132.5),(136.18,138.0)]),
]
for ref,pinnum,x,y,points in entries:
 orig=next(p for p in gps.Pads() if p.GetNumber()==pinnum)
 f=k.FootprintLoad(str(root/'FC_WirePad.pretty'),'WirePad_0p8_Drill0p5')
 f.SetFPIDAsString('FC_WirePad:WirePad_0p8_Drill0p5')
 f.SetReference(ref);f.SetValue('WIRE');f.SetAttributes(f.GetAttributes()|k.FP_BOARD_ONLY)
 f.SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y)))
 f.Value().SetVisible(False)
 f.Reference().SetTextSize(k.VECTOR2I(k.FromMM(.8),k.FromMM(.8)))
 f.Reference().SetTextThickness(k.FromMM(.13))
 f.Reference().SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y+1.6)))
 pad=next(iter(f.Pads()));b.Add(f);pad.SetNet(orig.GetNet())
 for (ax,ay),(bx,by) in zip(points,points[1:]):
  t=k.PCB_TRACK(b);t.SetStart(k.VECTOR2I(k.FromMM(ax),k.FromMM(ay)));t.SetEnd(k.VECTOR2I(k.FromMM(bx),k.FromMM(by)));t.SetWidth(k.FromMM(.35));t.SetLayer(k.B_Cu);t.SetNet(orig.GetNet());b.Add(t)
k.SaveBoard(str(w/'with-gps-wirepads.kicad_pcb'),b)
print('saved GPS pads')
