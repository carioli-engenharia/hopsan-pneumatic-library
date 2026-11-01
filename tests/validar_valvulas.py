from pathlib import Path
import subprocess,xml.etree.ElementTree as E,json,copy
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent.parent
lib=root/'Library';tests=lib/'testes'
cli=Path(os.environ['HOPSAN_ROOT'])/'bin'/('hopsancli.exe' if os.name=='nt' else 'hopsancli')
types=['CPC_Valvula32NF','CPC_Valvula32NA','CPC_Valvula52Mola','CPC_Valvula53Fechada','CPC_Valvula53Exaustao','CPC_Valvula53Pressao','CPC_Valvula52Memoria']
records=[]
def setp(tree,component,name,value):
    c=tree.find(f'./system/objects/component[@name="{component}"]/parameters')
    p=c.find(f'parameter[@name="{name}"]')
    if p is None:p=E.SubElement(c,'parameter',name=name,type='double',unit='')
    p.set('value',str(value))
def run(path,suffix='',step='hmf'):
    csv=tests/(path.stem+suffix+'.csv')
    p=subprocess.run([str(cli),'-e',str(lib/'pneumaticadidatica_v04.dll'),'-m',str(path),'-s',step,'--resultsFullCSV',str(csv),'--resultsCSVSort','cols','-l','801' if path.name.startswith('Exemplo_') else '201'],capture_output=True,text=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    log=p.stdout+p.stderr;(tests/(path.stem+suffix+'.log')).write_text(log,encoding='utf-8')
    assert p.returncode==0 and 'Error:'not in log and 'Warning:'not in log,log[-3500:]
    with csv.open()as f:names=f.readline().strip().split(',')
    a=np.loadtxt(csv,delimiter=',',skiprows=3);assert np.isfinite(a).all()
    d={n:a[:,j] for j,n in enumerate(names)}
    assert np.max(np.abs(d['Valvula#balanco_massa#Value']))<1e-12
    assert np.max(np.abs(d['Valvula#balanco_energia#Value']))<1e-8
    assert np.max(np.abs(d['Valvula#residuo#Value']))<1e-3
    return d
for mode,typ in enumerate(types):
    ports=['P','A','R']+(['B','S']if mode>=2 else[])
    for reverse in [False,True]:
      for u in [-1,0,1]:
        tree=E.parse(tests/(typ+'.hmf'));setp(tree,'Valvula','u#Value',u)
        if reverse:
            for pn in ports:setp(tree,'Fonte_'+pn,'pinput#Value',{'P':100000,'A':600000,'B':550000,'R':800000,'S':750000}[pn])
        path=tests/f'{typ}_u{u}_rev{int(reverse)}.hmf';tree.write(path,encoding='utf-8',xml_declaration=True)
        d=run(path);flows={pn:float(d[f'Valvula#{pn}#MassFlow'][-1])for pn in ports}
        # Independent truth table, checked by signs and exact closed ports.
        state=int(d['Valvula#estado#Value'][-1]);sg=-1 if reverse else 1
        if mode<2:
            supply=(u>=.5)if mode==0 else(u<.5)
            expected={'P':-sg,'A':sg,'R':0}if supply else{'P':0,'A':-sg,'R':sg}
        elif state==1:expected={'P':-sg,'A':sg,'B':-sg,'R':0,'S':sg}
        elif state==-1:expected={'P':-sg,'A':-sg,'B':sg,'R':sg,'S':0}
        elif mode==3:expected=dict.fromkeys(ports,0)
        elif mode==4:expected={'P':0,'A':-sg,'B':-sg,'R':sg,'S':sg}
        else:expected={'P':-sg,'A':sg,'B':sg,'R':0,'S':0}
        for pn in ports:assert int(np.sign(flows[pn]))==expected[pn],(typ,u,reverse,flows,expected)
        records.append({'tipo':typ,'ensaio':'estatico','u':u,'reverso':reverse,'estado':state,'vazoes_kg_s':flows,'aprovado':True})
    d=run(lib/('Exemplo_'+typ+'.hmf'))
    tail=d['Time']>7.9
    a=float(np.mean(d['Valvula#A#Pressure'][tail]));b=float(np.mean(d['Valvula#B#Pressure'][tail]))if mode>=2 else None
    targets=(600000,None)if mode==1 else (100000,None)if mode==0 else(100000,100000)if mode==4 else(600000,600000)if mode==5 else(100000,600000)
    # Closed centre retains the pressures present when the centre was selected.
    if mode!=3:
        assert abs(a-targets[0])<3000,(typ,a,targets)
        if b is not None:assert abs(b-targets[1])<3000,(typ,b,targets)
    else:
        for pn in ['A','B']:
            pp=d[f'Valvula#{pn}#Pressure'];assert np.ptp(pp[d['Time']>3.6])<1, (typ,pn,np.ptp(pp[d['Time']>3.6]))
    if mode==6:assert np.all(d['Valvula#estado#Value'][d['Time']>3.6]==-1)
    records.append({'tipo':typ,'ensaio':'dinamico','pressao_A_final_Pa':a,'pressao_B_final_Pa':b,'residuo_max_Pa':float(np.max(d['Valvula#residuo#Value'])),'aprovado':True})
    fig,ax=plt.subplots(2,1,figsize=(8,5),sharex=True,gridspec_kw={'height_ratios':[1,2]})
    ax[0].plot(d['Time'],d['Valvula#estado#Value'],color='#a45a00');ax[0].set_ylabel('Posição');ax[0].set_yticks([-1,0,1])
    ax[1].plot(d['Time'],d['Valvula#A#Pressure']/1e5,label='2 (A)')
    if b is not None:ax[1].plot(d['Time'],d['Valvula#B#Pressure']/1e5,label='4 (B)')
    ax[1].set_ylabel('Pressão absoluta (bar)');ax[1].set_xlabel('Tempo (s)');ax[1].legend()
    for aa in ax:aa.grid(alpha=.25)
    fig.suptitle(typ.replace('CPC_Valvula','Válvula '));fig.tight_layout();fig.savefig(lib/(typ+'_ensaio.png'),dpi=150);plt.close(fig)
    print(typ,'aprovada',round(a),None if b is None else round(b),flush=True)
    # Half time-step check on common sample times.
    half=run(lib/('Exemplo_'+typ+'.hmf'),'_meio_passo','0,0.00005,8')
    dif=max(float(np.max(np.abs(d[f'Valvula#{pn}#Pressure']-half[f'Valvula#{pn}#Pressure'])))for pn in ['A']+(['B']if mode>=2 else[]))
    assert dif<10000,(typ,dif)
    records.append({'tipo':typ,'ensaio':'meio_passo','desvio_max_pressao_Pa':dif,'aprovado':True})
(lib/'validacao_valvulas.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
print('TOTAL',len(records),'ensaios aprovados')

