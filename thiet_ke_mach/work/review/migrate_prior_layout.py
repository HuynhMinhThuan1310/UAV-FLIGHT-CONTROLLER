import pcbnew as k, pathlib, shutil

root=pathlib.Path.cwd(); oldroot=pathlib.Path(r'C:\Users\ADMIN\Downloads\mach_uav (1)')
w=root/'work/prior_layout_adapt';w.mkdir(exist_ok=True)
for name in ('mach_uav.kicad_sch','mach_uav.kicad_pro','fp-lib-table'):
 shutil.copy2(root/name,w/name)
for name in ('FC_PinHeaders.pretty','FC_WirePad.pretty'):
 shutil.copytree(root/name,w/name,dirs_exist_ok=True)
shutil.copytree(oldroot/'project_footprints/mach_uav_jumpers.pretty',w/'mach_uav_jumpers.pretty',dirs_exist_ok=True)
table=(w/'fp-lib-table').read_text()
table=table.rstrip()
table=table[:table.rfind(')')]+'\t(lib (name "mach_uav_jumpers") (type "KiCad") (uri "${KIPRJMOD}/mach_uav_jumpers.pretty") (options "") (descr "assembly jumper"))\n)\n'
(w/'fp-lib-table').write_text(table)

cur=k.LoadBoard(str(root/'mach_uav.kicad_pcb'))
b=k.LoadBoard(str(oldroot/'mach_uav.kicad_pcb'))
cf={f.GetReference():f for f in cur.GetFootprints()}
reroute={'/5V_BEC','/5V_BEC1','/5V','/USART1_RX','/USART1_TX','/USART3_RX','/USART3_TX'}
tracks=list(b.GetTracks())
for t in tracks:
 if t.GetNetname() in reroute:b.Remove(t)
k.SaveBoard(str(w/'no-old-nets.kicad_pcb'),b)

b=k.LoadBoard(str(w/'no-old-nets.kicad_pcb'))
zones=list(b.Zones())
for z in zones:
 if not z.GetIsRuleArea():b.Remove(z)
k.SaveBoard(str(w/'no-ground.kicad_pcb'),b)

b=k.LoadBoard(str(w/'no-ground.kicad_pcb'))
footprints=list(b.GetFootprints())
for f in footprints:
 if f.GetReference() not in cf and f.GetReference()!='W1':b.Remove(f)
k.SaveBoard(str(w/'pruned.kicad_pcb'),b)

b=k.LoadBoard(str(w/'pruned.kicad_pcb'))
for f in list(b.GetFootprints()):
 ref=f.GetReference()
 if ref=='W1':continue
 source=cf[ref]
 f.SetFPIDAsString(source.GetFPIDAsString())
 f.SetValue(source.GetValue())
 psrc={p.GetNumber():p for p in source.Pads()}
 for p in f.Pads():
  n=psrc[p.GetNumber()].GetNetname()
  if not n:
   p.SetNetCode(0)
  else:
   net=b.FindNet(n)
   if net is None:
    net=k.NETINFO_ITEM(b,n);b.Add(net)
   p.SetNet(net)

k.SaveBoard(str(w/'mach_uav.kicad_pcb'),b)
print('saved candidate')
