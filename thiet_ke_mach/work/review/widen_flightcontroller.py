import pcbnew as k, pathlib, json, subprocess, collections

root=pathlib.Path.cwd(); work=root/'work/flightcontroller'
src=work/'candidate.kicad_pcb'; out=work/'wide.kicad_pcb'
b=k.LoadBoard(str(src))
for t in b.GetTracks(): t.SetWidth(k.FromMM(.762))
cli=r'C:\Program Files\KiCad\10.0\bin\kicad-cli.exe'
for i in range(12):
 k.SaveBoard(str(out),b)
 subprocess.run([cli,'pcb','drc',str(out),'-o',str(work/'wide-drc.json'),'--format','json'],capture_output=True,check=False)
 d=json.loads((work/'wide-drc.json').read_text(encoding='utf-8'))
 bad=set()
 for v in d['violations']:
  if v['type'] in ('clearance','shorting','copper_edge_clearance','hole_clearance','solder_mask_bridge'):
   bad.update(x['uuid'] for x in v['items'] if 'uuid' in x)
 tracks={str(t.m_Uuid.AsString()):t for t in b.GetTracks()}
 shrink=[tracks[u] for u in bad if u in tracks and k.ToMM(tracks[u].GetWidth())>.351]
 print('pass',i,'violations',collections.Counter(v['type'] for v in d['violations']),'shrink',len(shrink),flush=True)
 if not shrink: break
 for t in shrink: t.SetWidth(k.FromMM(.35))
else: raise RuntimeError('Unable to settle trace clearance')
k.SaveBoard(str(out),b)
print('widths',dict(collections.Counter(round(k.ToMM(t.GetWidth()),3) for t in b.GetTracks())))
