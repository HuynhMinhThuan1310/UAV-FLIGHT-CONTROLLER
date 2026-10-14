import pathlib,sys
src=pathlib.Path('work/review/route_single.py').read_text(encoding='utf-8-sig')
variant=int(sys.argv[1]) if len(sys.argv)>1 else 0
src=src.replace("ROOT/'work'/'single_layer'",f"ROOT/'work'/'no_ground_{variant}'").replace('STEP=.2','STEP=.1')
src=src.replace("'J23':(87,73,90)","'J23':(84,73,90)").replace("'J32':(119,98,0)",f"'J32':(119,98,{variant*90})")
src=src.replace('width={n:',"nets=[n for n in nets if names[n] not in ['G','GND','/GND']]\nwidth={n:")
exec(src[:src.index('best=999;')])
exec(pathlib.Path('work/review/fast_search.txt').read_text(encoding='utf-8-sig'))
def astar(block,start,targets):return search(block,np.zeros((NY,NX),dtype=np.float32),start,targets)
span=lambda n:max(p[1] for p in netpads[n])-min(p[1] for p in netpads[n])+max(p[2] for p in netpads[n])-min(p[2] for p in netpads[n])
random.seed(473+variant);best=999;priority=set()
for attempt in range(100):
 if attempt==0:order=sorted(nets,key=lambda n:(len(netpads[n])>3,span(n)))
 elif attempt%3==0:order=sorted(nets,key=lambda n:span(n)*random.uniform(.3,2.5))
 else:
  order=nets.copy();random.shuffle(order)
  if attempt%3==1:order.sort(key=lambda n:n not in priority)
 paths,failed=route(order);priority={n for n,_,_ in failed}
 if len(failed)<best:
  best=len(failed);save(paths,failed,attempt)
 if attempt%10==0:print('TRY',attempt,'BEST',best,flush=True)
 if best==0:break
