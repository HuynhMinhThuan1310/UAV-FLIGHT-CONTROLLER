import pathlib,sys
src=pathlib.Path('work/review/route_single.py').read_text(encoding='utf-8-sig')
src=src.replace("'J23':(87,73,90)","'J23':(84,73,90)")
# Connect short local branches across all nets before long trunk routes.
def branch_route():
 edges=[]
 for n,pp in netpads.items():
  if n not in nets:continue
  links=sorted((math.hypot(p[1]-q[1],p[2]-q[2]),i,j) for i,p in enumerate(pp) for j,q in enumerate(pp) if j>i)
  parent=list(range(len(pp)))
  def find(i):
   while parent[i]!=i:i=parent[i]
   return i
  for length,i,j in links:
   a,bb=find(i),find(j)
   if a==bb:continue
   parent[a]=bb;edges.append((length*random.uniform(.6,1.6),n,i,j))
 edges.sort();done={};paths={};failed=[];components={n:[{p[0]} for p in netpads[n]] for n in nets}
 for _,n,i,j in edges:
  obstacle=base[n].copy()
  for other,cc in done.items():
   if other!=n:stamp(obstacle,cc,(width[n]+width[other])/2+.20+.04)
  groups=components[n];a=next(g for g in groups if netpads[n][i][0] in g);bb=next(g for g in groups if netpads[n][j][0] in g)
  if a is bb:continue
  target=bb;start=min(a,key=lambda c:min(abs(c%NX-t%NX)+abs(c//NX-t//NX) for t in target))
  path=astar(obstacle,start,target)
  if path is None:failed.append((n,netpads[n][j][3],netpads[n][j][4]));continue
  paths.setdefault(n,[]).append(path);done.setdefault(n,set()).update(path);a.update(bb);a.update(path);groups.remove(bb)
 return paths,failed
bestglobal=999
for variant in range(4):
 ss=src.replace("'J32':(119,98,0)",f"'J32':(119,98,{variant*90})")
 exec(ss[:ss.index('best=999;')])
 random.seed(213+variant)
 for attempt in range(6):
  paths,failed=branch_route();print('VAR',variant,'ATTEMPT',attempt,'FAILED',len(failed),flush=True)
  if len(failed)<bestglobal:
   bestglobal=len(failed);save(paths,failed,variant*10+attempt)
   shutil.copy2(W/'candidate.kicad_pcb',W/'best-branch.kicad_pcb')
   shutil.copy2(W/'routing.json',W/'best-branch.json')
  if not failed:break
 if bestglobal==0:break
