from pathlib import Path
import subprocess,xml.etree.ElementTree as E,json,copy
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent.parent;lib=root/'Library';out=lib/'testes_acessorios';out.mkdir(exist_ok=True)
circuits=lib/'Circuitos';cli=Path(os.environ['HOPSAN_ROOT'])/'bin'/('hopsancli.exe' if os.name=='nt' else 'hopsancli')
base=E.parse(circuits/'06_Regulador_Reservatorio.hmf').getroot();records=[]
def new(name):
    r=copy.deepcopy(base);s=r.find('system');s.set('name',name);o=s.find('objects');o.clear();c=s.find('connections');c.clear();return r,o,c
def add(o,typ,name,params={}):
    c=E.SubElement(o,'component',typename=typ,name=name,disabled='false');p=E.SubElement(c,'parameters')
    for k,v in params.items():E.SubElement(p,'parameter',name=k+'#Value',value=str(v),type='double',unit='')
def wire(c,a,ap,b,bp):E.SubElement(c,'connect',startcomponent=a,startport=ap,endcomponent=b,endport=bp)
def run(path,step='0,.00005,4',suffix='',folder=out):
    csv=folder/(path.stem+suffix+'.csv')
    start,dt,stop=map(float,step.split(','));samples=str(min(2001,int(round((stop-start)/dt))+1))
    p=subprocess.run([str(cli),'-e',str(lib/'pneumaticadidatica_v04.dll'),'-m',str(path),'-s',step,'--resultsFullCSV',str(csv),'--resultsCSVSort','cols','-l',samples],capture_output=True,text=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    log=p.stdout+p.stderr;(folder/(path.stem+suffix+'.log')).write_text(log,encoding='utf-8')
    assert p.returncode==0 and 'Error:' not in log and 'Warning:' not in log,(path.name,log[-2500:])
    names=csv.read_text().splitlines()[0].split(',');a=np.loadtxt(csv,delimiter=',',skiprows=3);assert np.isfinite(a).all()
    d={n:a[:,i] for i,n in enumerate(names)}
    for k,v in d.items():
        if '#balanco_massa#' in k:assert max(abs(v))<1e-12
        if '#balanco_energia#' in k:assert max(abs(v))<1e-8
        if '#residuo#' in k:assert max(abs(v))<1e-3
    return d
def execute(r,name,step='0,.00005,.002'):
    path=out/(name+'.hmf');E.ElementTree(r).write(path,encoding='utf-8',xml_declaration=True);return run(path,step)

# Independent expected flow directions; cases include blocked/reverse/cracking limits.
cases=[
('Estranguladora_aberta','PD_Estranguladora',{'P':600000,'A':100000},{'u':1},{'P':-1,'A':1}),
('Estranguladora_fechada','PD_Estranguladora',{'P':600000,'A':100000},{'u':0},{'P':0,'A':0}),
('Estranguladora_reversa','PD_Estranguladora',{'P':100000,'A':600000},{'u':1},{'P':1,'A':-1}),
('Retencao_direta','PD_Retencao',{'P':600000,'A':100000},{},{'P':-1,'A':1}),
('Retencao_reversa','PD_Retencao',{'P':100000,'A':600000},{},{'P':0,'A':0}),
('Retencao_abaixo_abertura','PD_Retencao',{'P':100500,'A':100000},{},{'P':0,'A':0}),
('Unidirecional_direta','PD_ReguladoraUnidirecional',{'P':600000,'A':100000},{},{'P':-1,'A':1}),
('Unidirecional_reversa','PD_ReguladoraUnidirecional',{'P':100000,'A':600000},{},{'P':1,'A':-1}),
('Escape_alimenta','PD_EscapeRapido',{'P':600000,'A':100000,'R':100000},{},{'P':-1,'A':1,'R':0}),
('Escape_descarrega','PD_EscapeRapido',{'P':100000,'A':600000,'R':100000},{},{'P':0,'A':-1,'R':1}),
('Regulador_alimenta','PD_ReguladorPressao',{'P':600000,'A':200000,'R':100000},{},{'P':-1,'A':1,'R':0}),
('Regulador_alivia','PD_ReguladorPressao',{'P':600000,'A':500000,'R':100000},{},{'P':0,'A':-1,'R':1}),
('Regulador_equilibrio','PD_ReguladorPressao',{'P':600000,'A':400000,'R':100000},{},{'P':0,'A':0,'R':0}),
('Alivio_aberto','PD_Alivio',{'P':600000,'A':100000},{},{'P':-1,'A':1}),
('Alivio_fechado','PD_Alivio',{'P':300000,'A':100000},{},{'P':0,'A':0}),
('Bloqueio_aberto','PD_Bloqueio',{'P':600000,'A':100000},{'u':1},{'P':-1,'A':1}),
('Bloqueio_fechado','PD_Bloqueio',{'P':600000,'A':100000},{'u':0},{'P':0,'A':0}),
]
flow_results={}
for name,typ,pressures,params,expected in cases:
    r,o,c=new(name);add(o,typ,'Valvula',params)
    for port,pressure in pressures.items():
        add(o,'PneumaticPTsrc','Fonte_'+port,{'pinput':pressure,'Tinput':293});wire(c,'Fonte_'+port,'Pp1','Valvula',port)
    d=execute(r,name);flows={p:float(d[f'Valvula#{p}#MassFlow'][-1]) for p in pressures}
    assert {p:int(np.sign(v)) for p,v in flows.items()}==expected,(name,flows,expected)
    flow_results[name]=flows;records.append({'ensaio':name,'aprovado':True,'vazoes_kg_s':flows})
ratio=flow_results['Unidirecional_reversa']['P']/-flow_results['Unidirecional_direta']['P'];assert abs(ratio-4)<1e-5
records.append({'ensaio':'Razao de areas da reguladora unidirecional','razao_vazao':ratio,'aprovado':True})

# Four pressure combinations for each pneumatic logic gate.
for typ,kind in [('PD_SeletoraOU','OU'),('PD_DuasPressoesE','E')]:
    for a,b in [(0,0),(0,1),(1,0),(1,1)]:
        name=f'Logica_{kind}_{a}{b}';r,o,c=new(name);add(o,typ,'Valvula');add(o,'PD_Reservatorio','Carga',{'Volume':.00001})
        wire(c,'Valvula','A','Carga','P')
        for port,on in [('X',a),('Y',b)]:add(o,'PneumaticPTsrc','Fonte_'+port,{'pinput':600000 if on else 100000,'Tinput':293});wire(c,'Fonte_'+port,'Pp1','Valvula',port)
        d=execute(r,name,'0,.00001,1');target=600000 if ((a or b) if kind=='OU' else (a and b)) else 100000
        actual=float(d['Carga#p_abs#Value'][-1]);assert abs(actual-target)<1000,(name,actual,target)
        records.append({'ensaio':name,'pressao_final_Pa':actual,'aprovado':True})

# Signal control truth table.
for a,b,expected,conflict in [(0,0,0,0),(1,0,1,0),(0,1,-1,0),(1,1,0,1)]:
    name=f'Bobinas_{a}{b}';r,o,c=new(name);add(o,'PD_ComandoBobinas','Bobinas',{'avancar':a,'recuar':b});d=execute(r,name)
    assert d['Bobinas#u#Value'][-1]==expected and d['Bobinas#conflito#Value'][-1]==conflict
    records.append({'ensaio':name,'aprovado':True})
for x,back,front in [(0,1,0),(.05,0,0),(.1,0,1)]:
    name=f'FimCurso_{x}';r,o,c=new(name);add(o,'PD_FinsCurso','Sensor',{'x':x});d=execute(r,name)
    assert d['Sensor#recuado#Value'][-1]==back and d['Sensor#avancado#Value'][-1]==front
    records.append({'ensaio':name,'aprovado':True})
r,o,c=new('TON_reset');add(o,'SignalPulse','Pulso',{'t_start':.2,'t_end':1,'y_0':0,'y_A':1});add(o,'PD_TemporizadorTON','TON',{'Atraso':.3});wire(c,'Pulso','out','TON','in');d=execute(r,'TON_reset','0,.00005,1.5')
t=d['Time'];v=d['TON#out#Value'];turn=float(t[np.where(v>.5)[0][0]]);assert abs(turn-.5)<.002 and v[-1]==0 and d['TON#decorrido#Value'][-1]==0
records.append({'ensaio':'TON atraso e reset','ligou_s':turn,'aprovado':True})

# Six runnable user circuits, including half-step sensitivity and pressure readout.
for path in sorted(circuits.glob('*.hmf')):
    if not path.stem[:2] in ['05','06','07','08','09','10']:continue
    d=run(path,folder=circuits);half=run(path,'0,.000025,4','_meio_passo',folder=circuits)
    if path.stem.startswith('05'):
        key='Cilindro#x#Value';y=d[key]*1000;unit='Posição (mm)';assert max(y)>99 and y[-1]<.5,(path.name,max(y),y[-1]);assert min(d['Cilindro#p_A#Value'])>0
        diff=float(max(abs(y-half[key]*1000)));assert diff<.5
        rec={'curso_max_mm':float(max(y)),'posicao_final_mm':float(y[-1]),'desvio_meio_passo_mm':diff}
    else:
        key='Reservatorio#p_abs#Value';y=d[key]/1e5;unit='Pressão absoluta (bar)';diff=float(max(abs(y-half[key]/1e5)));assert diff<.02,(path.name,diff)
        target={'06':4,'07':6,'08':4.5,'09':1,'10':5.99}[path.stem[:2]];assert abs(y[-1]-target)<.03,(path.name,y[-1],target)
        measured=d['Manometro#p_abs#Value'];assert max(abs(measured[2:]-d[key][2:]))<300,(path.name,'readout')
        rec={'pressao_final_bar_abs':float(y[-1]),'desvio_meio_passo_bar':diff,'pressostato_final':float(d['Manometro#ligado#Value'][-1])}
    fig,ax=plt.subplots(figsize=(8,4));ax.plot(d['Time'],y);ax.set_xlabel('Tempo (s)');ax.set_ylabel(unit);ax.grid(alpha=.25);fig.tight_layout();fig.savefig(circuits/(path.stem+'.png'),dpi=150);plt.close(fig)
    records.append({'ensaio':path.stem,'aprovado':True,**rec});print(path.stem,rec,flush=True)
(lib/'validacao_acessorios.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
print(len(records),'ensaios de acessórios e controles aprovados.')
