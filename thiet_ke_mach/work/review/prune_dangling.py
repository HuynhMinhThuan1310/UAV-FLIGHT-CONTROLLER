import pcbnew as k, pathlib, subprocess, json, os

os.dup2(os.open(os.devnull,os.O_WRONLY),2)
w=pathlib.Path.cwd()/'work/prior_layout_adapt'
board=w/'mach_uav.kicad_pcb'
cli=r'C:\Program Files\KiCad\10.0\bin\kicad-cli.exe'
subprocess.run([cli,'pcb','drc',str(board),'--schematic-parity','-o',str(w/'drc.json'),'--format','json'],capture_output=True)
d=json.loads((w/'drc.json').read_text())
bad={x['uuid'] for v in d['violations'] if v['type']=='track_dangling' for x in v['items'] if 'uuid' in x}
if not bad:
 print('DONE')
else:
 b=k.LoadBoard(str(board));tracks=list(b.GetTracks())
 count=0
 for t in tracks:
  if t.m_Uuid.AsString() in bad:b.Remove(t);count+=1
 if not count:raise RuntimeError('No matching tracks')
 k.SaveBoard(str(board),b)
 print('REMOVED',count,flush=True)
