import pcbnew as k, json, collections, pathlib
b=k.LoadBoard('mach_uav.kicad_pcb')
print('board', b.GetFileName())
print('footprints',len(list(b.GetFootprints())),'tracks',len(list(b.GetTracks())),'zones',len(list(b.Zones())))
print('edges',[(str(d.GetClass()),d.GetLayerName()) for d in b.GetDrawings() if d.GetLayer()==k.Edge_Cuts][:15])
for f in sorted(b.GetFootprints(),key=lambda x:x.GetReference()):
 p=f.GetPosition(); pads=list(f.Pads()); print(f.GetReference(),f.GetValue(),f.GetFPIDAsString(),tuple(round(k.ToMM(v),2) for v in [p.x,p.y]),f.GetOrientationDegrees(),f.GetLayerName(),len(pads),'pads',[(x.GetNumber(), x.GetNetname()) for x in pads][:12])
print('track layers',collections.Counter(t.GetLayerName() for t in b.GetTracks()))
