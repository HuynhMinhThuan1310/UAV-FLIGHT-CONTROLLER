import pcbnew as k, pathlib, hashlib, json, shutil
root=pathlib.Path.cwd();w=root/'work/single_layer'
assert hashlib.sha256((root/'mach_uav.kicad_pcb').read_bytes()).digest()==hashlib.sha256((w/'original.kicad_pcb').read_bytes()).digest(), 'Original PCB changed while routing; refusing to overwrite.'
b=k.LoadBoard(str(w/'best-branch.kicad_pcb'))
for f in b.GetFootprints():
 r=f.GetReference()
 if r in ['J8','J15','J16','J17']:
  f.Reference().SetPosition(k.VECTOR2I(k.FromMM(77),f.GetPosition().y))
 if r=='J18': f.Reference().SetPosition(k.VECTOR2I(k.FromMM(77.5),k.FromMM(115.5)))
 if r=='J26': f.Reference().SetPosition(k.VECTOR2I(k.FromMM(121),k.FromMM(121)))
assert not any(isinstance(t,k.PCB_VIA) for t in b.GetTracks())
assert all(t.GetLayer()==k.B_Cu for t in b.GetTracks())
k.SaveBoard(str(w/'final-draft.kicad_pcb'),b)
shutil.copy2(root/'mach_uav.kicad_pcb',root/'work/single_layer/mach_uav_before_single_layer.kicad_pcb')
shutil.copy2(w/'final-draft.kicad_pcb',root/'mach_uav.kicad_pcb')
report={'status':'DRAFT - NOT READY FOR FABRICATION','size_mm':[58,64],'routing_layer':'B.Cu','vias':0,'front_tracks':0,'tracks':len(list(b.GetTracks())),'remaining_connections':json.loads((w/'best-branch.json').read_text())['failed'],'known_limits':['10 unconnected PCB items before final check','Header pad spacing is 0.15 mm; original project requires 0.20 mm. No clearance rules were relaxed.','Schematic still has SBUS, ADC0, GPIO2, SERVO and AUX_PWM connectivity issues identified earlier.','GPS SMD connector moved to bottom side.','Power trace widths are not validated against load current.'],'backup':'work/single_layer/mach_uav_before_single_layer.kicad_pcb'}
(w/'final-summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
