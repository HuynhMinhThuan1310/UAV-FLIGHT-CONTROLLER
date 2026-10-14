import pcbnew as k, pathlib,os
os.dup2(os.open(os.devnull,os.O_WRONLY),2)
w=pathlib.Path.cwd()/'work/prior_layout_adapt'
b=k.LoadBoard(str(w/'routed.kicad_pcb'))
old=k.LoadBoard(r'C:\Users\ADMIN\Downloads\mach_uav (1)\mach_uav.kicad_pcb')
code=b.FindNet('/5V_BEC').GetNetCode();n=0
for t in old.GetTracks():
 if t.GetNetname()!='/5V_BEC':continue
 c=t.Duplicate();c.SetNetCode(code);b.Add(c);n+=1
k.SaveBoard(str(w/'routed-plus-bec.kicad_pcb'),b)
print('copied',n)
