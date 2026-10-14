import pcbnew as k,pathlib,hashlib,json,shutil
root=pathlib.Path.cwd();w=root/'work/no_ground_refined';current=root/'mach_uav.kicad_pcb';snapshot=root/'work/no_ground_1/original.kicad_pcb'
assert hashlib.sha256(current.read_bytes()).digest()==hashlib.sha256(snapshot.read_bytes()).digest(),'PCB changed during routing; original not overwritten.'
b=k.LoadBoard(str(w/'candidate.kicad_pcb'))
for f in b.GetFootprints():
 r=f.GetReference()
 if r in ['J8','J15','J16','J17']: f.Reference().SetPosition(k.VECTOR2I(k.FromMM(77),f.GetPosition().y))
 if r=='J18':f.Reference().SetPosition(k.VECTOR2I(k.FromMM(77.5),k.FromMM(115.5)))
 if r=='J26':f.Reference().SetPosition(k.VECTOR2I(k.FromMM(121),k.FromMM(121)))
tracks=list(b.GetTracks())
assert not any(isinstance(t,k.PCB_VIA) for t in tracks)
assert all(t.GetLayer()==k.B_Cu and t.GetNetname() not in ['G','GND','/GND'] for t in tracks)
assert len(list(b.Zones()))==0
k.SaveBoard(str(w/'final.kicad_pcb'),b)
shutil.copy2(current,w/'before_no_ground.kicad_pcb');shutil.copy2(w/'final.kicad_pcb',current)
summary={'size_mm':[58,64],'tracks':len(tracks),'layer':'B.Cu','vias':0,'ground_tracks':0,'zones':0,'non_ground_unconnected':0,'ground_unconnected':17,'known_pad_clearance_mm':.15,'project_clearance_mm':.2}
(w/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary))
