import pcbnew as k,pathlib,os
os.dup2(os.open(os.devnull,os.O_WRONLY),2)
w=pathlib.Path.cwd()/'work/prior_layout_adapt'
b=k.LoadBoard(str(w/'with-gps-wirepads.kicad_pcb'))
lines=[
 'WIRE NGOAI (NOI THEO SO CHAN)',
 'W1 MOSI  U3.33 - J32.6',
 'W2 5V_BEC  J1.2 - U3.38',
 'W3 USART1_RX  J18.2 - U3.27',
 'W4 USART1_TX  J18.3 - U3.26',
 'W5 USART3_RX  U3.16 - W5B',
 'W6 USART3_TX  U3.15 - W6B',
 'G: CHUA DI, DO MASS SAU',
]
for i,s in enumerate(lines):
 t=k.PCB_TEXT(b);t.SetText(s);t.SetLayer(k.F_SilkS)
 t.SetTextSize(k.VECTOR2I(k.FromMM(.8),k.FromMM(.8)))
 t.SetTextThickness(k.FromMM(.14));t.SetHorizJustify(k.GR_TEXT_H_ALIGN_LEFT)
 t.SetPosition(k.VECTOR2I(k.FromMM(72),k.FromMM(132+i*1.6)))
 b.Add(t)
k.SaveBoard(str(w/'annotated.kicad_pcb'),b)
print('annotations added')
