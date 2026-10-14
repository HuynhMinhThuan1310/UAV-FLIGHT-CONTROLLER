import pathlib
src=pathlib.Path('work/review/route_single.py').read_text(encoding='utf-8-sig').replace('STEP=.2','STEP=.1').replace("'J23':(87,73,90)","'J23':(84,73,90)")
bestglobal=999
for variant in range(4):
 ss=src.replace("'J32':(119,98,0)",f"'J32':(119,98,{variant*90})")
 exec(ss[:ss.index('best=999;')]); exec(pathlib.Path('work/review/fast_search.txt').read_text(encoding='utf-8-sig'))
 def astar(block,start,targets):return search(block,np.zeros((NY,NX),dtype=np.float32),start,targets)
 random.seed(584+variant);priority=set()
 for attempt in range(20):
  order=nets.copy();random.shuffle(order)
  if attempt%3:order.sort(key=lambda n:n not in priority)
  paths,failed=route(order);priority={n for n,_,_ in failed}
  print('V',variant,'A',attempt,'FAILED',len(failed),flush=True)
  if len(failed)<bestglobal:
   bestglobal=len(failed);save(paths,failed,variant*100+attempt)
   shutil.copy2(W/'candidate.kicad_pcb',W/'best-strict.kicad_pcb');shutil.copy2(W/'routing.json',W/'best-strict.json')
  if bestglobal==0:break
 if bestglobal==0:break
