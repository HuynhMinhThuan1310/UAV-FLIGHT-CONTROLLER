import pcbnew as k,pathlib,subprocess,json,collections,os
os.dup2(os.open(os.devnull,os.O_WRONLY),2)
w=pathlib.Path.cwd()/'work/prior_layout_adapt'
b=k.LoadBoard(str(w/'wide_bec1.kicad_pcb'))
for t in b.GetTracks():
 if t.GetNetname()=='/5V_BEC1' and k.ToMM(t.GetWidth())<.6:t.SetWidth(k.FromMM(.762))
out=w/'wide_bec1_30mil.kicad_pcb';cli=r'C:\Program Files\KiCad\10.0\bin\kicad-cli.exe'
for i in range(10):
 k.SaveBoard(str(out),b)
 subprocess.run([cli,'pcb','drc',str(out),'-o',str(w/'wide_bec1_30mil_drc.json'),'--format','json'],capture_output=True)
 d=json.loads((w/'wide_bec1_30mil_drc.json').read_text())
 bad={x['uuid'] for v in d['violations'] if v['type'] in ('clearance','shorting_items','solder_mask_bridge','copper_edge_clearance','tracks_crossing') for x in v['items'] if 'uuid' in x}
 tracks={t.m_Uuid.AsString():t for t in b.GetTracks() if t.GetNetname()=='/5V_BEC1'}
 shrink=[tracks[u] for u in bad if u in tracks and k.ToMM(tracks[u].GetWidth())>.5]
 print('pass',i,'violations',dict(collections.Counter(v['type'] for v in d['violations'])),'shrink',len(shrink),flush=True)
 if not shrink:break
 for t in shrink:t.SetWidth(k.FromMM(.5))
else:raise RuntimeError('Width convergence failed')
k.SaveBoard(str(out),b)
print('widths',dict(collections.Counter(round(k.ToMM(t.GetWidth()),3) for t in b.GetTracks() if t.GetNetname()=='/5V_BEC1')))

