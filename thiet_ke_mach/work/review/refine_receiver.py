import pathlib,itertools
src=pathlib.Path('work/review/route_single.py').read_text(encoding='utf-8-sig').replace("ROOT/'work'/'single_layer'","ROOT/'work'/'no_ground_refined'").replace('STEP=.2','STEP=.1').replace("'J23':(87,73,90)","'J23':(84,73,90)").replace("'J32':(119,98,0)","'J32':(119,98,90)")
src=src.replace('width={n:',"nets=[n for n in nets if names[n] not in ['G','GND','/GND']]\nwidth={n:")
exec(src[:src.index('best=999;')]);exec(pathlib.Path('work/review/fast_search.txt').read_text(encoding='utf-8-sig'))
def astar(block,start,targets):return search(block,np.zeros((NY,NX),dtype=np.float32),start,targets)
fixed={};oldtracks=json.loads(pathlib.Path('work/no_ground_1/tracks.json').read_text());receivers=[n for n in nets if 'RECV' in names[n]]
for t in oldtracks:
 n=b.FindNet(t['net']).GetNetCode()
 if n in receivers:continue
 x,y=t['start'];xx,yy=t['end'];count=max(2,math.ceil(math.hypot(xx-x,yy-y)/STEP*2))
 for fraction in np.linspace(0,1,count):
  cx,cy=coords(x+fraction*(xx-x),y+fraction*(yy-y));fixed.setdefault(n,set()).add(cy*NX+cx)
perms=list(itertools.permutations(receivers));random.seed(178);random.shuffle(perms);best=999
for iteration,order in enumerate(perms):
 done={n:cc.copy() for n,cc in fixed.items()};paths={};failed=[]
 for n in order:
  obstacle=base[n].copy()
  for other,cc in done.items():stamp(obstacle,cc,(width[n]+width[other])/2+.20+.04)
  pp=netpads[n];path=astar(obstacle,pp[1][0],{pp[0][0]})
  if path is None:failed.append((n,pp[1][3],pp[1][4]))
  else:paths[n]=[path];done[n]=set(path)
 if len(failed)<best:
  best=len(failed);bestpaths=paths;bestfailed=failed
  print('BEST',best,iteration,flush=True)
 if best==0:break
 if iteration%100==0:print('TRY',iteration,flush=True)
save(bestpaths,bestfailed,iteration)
for old in oldtracks:
 n=b.FindNet(old['net']).GetNetCode()
 if n in receivers:continue
 t=k.PCB_TRACK(b);t.SetNetCode(n);t.SetLayer(k.B_Cu);t.SetWidth(k.FromMM(old['width']));t.SetStart(k.VECTOR2I(k.FromMM(old['start'][0]),k.FromMM(old['start'][1])));t.SetEnd(k.VECTOR2I(k.FromMM(old['end'][0]),k.FromMM(old['end'][1])));b.Add(t)
k.SaveBoard(str(W/'candidate.kicad_pcb'),b)
