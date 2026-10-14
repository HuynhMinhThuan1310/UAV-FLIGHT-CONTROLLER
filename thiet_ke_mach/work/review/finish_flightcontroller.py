import pcbnew as k, pathlib, json

root=pathlib.Path.cwd(); work=root/'work/flightcontroller'
b=k.LoadBoard(str(work/'wide.kicad_pcb'))
fp={f.GetReference():f for f in b.GetFootprints()}
for f in fp.values():
 if f.GetReference().startswith('J') and f.GetReference()!='J26':
  f.SetFPIDAsString(f.GetFPIDAsString().replace('Connector_PinHeader_1.00mm:', 'FC_PinHeaders:'))
 f.Value().SetVisible(False)
 if f.GetReference()=='J26':
  f.Reference().SetPosition(k.VECTOR2I(k.FromMM(119),k.FromMM(128.5)))

wirepads=[('W3B','J26','4',119.5,132)]
for ref,source,pin,x,y in wirepads:
 orig=next(p for p in fp[source].Pads() if p.GetNumber()==pin)
 f=k.FootprintLoad(str(root/'FC_WirePad.pretty'),'WirePad_0p8_Drill0p5')
 if f is None: raise RuntimeError('wire pad footprint load failed')
 f.SetReference(ref); f.SetValue('WIRE')
 f.SetAttributes(f.GetAttributes() | k.FP_BOARD_ONLY)
 f.SetFPIDAsString('FC_WirePad:WirePad_0p8_Drill0p5')
 f.SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y)))
 f.Reference().SetTextSize(k.VECTOR2I(k.FromMM(.9),k.FromMM(.9)))
 f.Reference().SetTextThickness(k.FromMM(.15))
 f.Reference().SetPosition(k.VECTOR2I(k.FromMM(x),k.FromMM(y-1.5)))
 f.Value().SetVisible(False)
 pad=next(iter(f.Pads()))
 b.Add(f)
 pad.SetNet(orig.GetNet())
 t=k.PCB_TRACK(b); t.SetStart(orig.GetPosition());t.SetEnd(pad.GetPosition());t.SetWidth(k.FromMM(.35));t.SetLayer(k.B_Cu);t.SetNetCode(orig.GetNetCode());b.Add(t)

k.SaveBoard(str(work/'with-wirepads.kicad_pcb'),b)
print('footprints',len(list(b.GetFootprints())),'tracks',len(list(b.GetTracks())))
