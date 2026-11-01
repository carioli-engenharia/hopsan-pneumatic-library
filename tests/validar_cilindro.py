from pathlib import Path
import subprocess,json,xml.etree.ElementTree as E
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent.parent;lib=root/'Library';out=lib/'Circuitos'
cli=Path(os.environ['HOPSAN_ROOT'])/'bin'/('hopsancli.exe' if os.name=='nt' else 'hopsancli')
def run(path,suffix='',step='0,0.00005,2'):
    csv=out/(path.stem+suffix+'.csv')
    p=subprocess.run([str(cli),'-e',str(lib/'pneumaticadidatica_v04.dll'),'-m',str(path),'-s',step,'--resultsFullCSV',str(csv),'--resultsCSVSort','cols','-l','2001'],capture_output=True,text=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    log=p.stdout+p.stderr;(out/(path.stem+suffix+'.log')).write_text(log,encoding='utf-8')
    assert p.returncode==0 and 'Error:' not in log and 'Warning:' not in log,log[-3000:]
    names=csv.read_text().splitlines()[0].split(',');a=np.loadtxt(csv,delimiter=',',skiprows=3);assert np.isfinite(a).all()
    return {name:a[:,i] for i,name in enumerate(names)}
records=[]
for path in sorted(out.glob('0[1-4]_*.hmf')):
    d=run(path);r=run(path,'_meio_passo','0,0.000025,2');t=d['Time']
    get=lambda key:d['Cilindro_1#'+key+'#Value']
    x=get('x');v=get('v');pA=get('p_A');pB=get('p_B')
    diff=np.max(abs(x-r['Cilindro_1#x#Value']))
    assert x.min()>=0 and x.max()<=.1 and min(pA.min(),pB.min())>0
    assert diff<.001, (path,diff)
    rec={'circuito':path.stem,'aprovado':True,'x_min_mm':float(1000*x.min()),'x_max_mm':float(1000*x.max()),'x_final_mm':float(1000*x[-1]),'diferenca_meio_passo_mm':float(1000*diff),'residuo_energia_max_J':float(max(abs(get('residuo_energia')))),'pressao_A_max_bar_abs':float(pA.max()/1e5),'pressao_B_max_bar_abs':float(pB.max()/1e5)}
    if 'Fechado' in path.stem:
        mask=(t>=.5)&(t<=.95)
        rec['variacao_massa_A_centro_kg']=float(np.ptp(get('massa_A')[mask]));rec['variacao_massa_B_centro_kg']=float(np.ptp(get('massa_B')[mask]))
        assert rec['variacao_massa_A_centro_kg']<1e-12 and rec['variacao_massa_B_centro_kg']<1e-12
        rec['x_inicio_centro_mm']=float(1000*np.interp(.32,t,x));rec['x_fim_centro_mm']=float(1000*np.interp(.95,t,x))
    records.append(rec)
    fig,axs=plt.subplots(3,1,figsize=(9,7),sharex=True)
    axs[0].plot(t,1000*x);axs[0].set_ylabel('Posição (mm)')
    axs[1].plot(t,pA/1e5,label='Câmara A');axs[1].plot(t,pB/1e5,label='Câmara B');axs[1].set_ylabel('Pressão absoluta (bar)');axs[1].legend()
    axs[2].plot(t,d['Comando#u#Value']);axs[2].set_ylabel('Comando u');axs[2].set_xlabel('Tempo (s)')
    for ax in axs:ax.grid(alpha=.25)
    fig.tight_layout();fig.savefig(out/(path.stem+'.png'),dpi=150);plt.close(fig)
    print(rec,flush=True)
# Independent sealed-gas test: no flows, no damping; small initial velocity.
p=out/'Teste_Adiabatico.hmf';tree=E.parse(out/'01_53_Centro_Fechado.hmf');s=tree.find('system');objs=s.find('objects');objs.remove(objs.find("component[@name='Comando']"));cons=s.find('connections')
for con in list(cons):
    if con.get('startcomponent')=='Comando':cons.remove(con)
pars=objs.find("component[@name='Cilindro_1']/parameters")
for name,val in [('B_viscoso#Value',0),('v_inicial#Value',.1)]:
    par=pars.find(f"parameter[@name='{name}']")
    if par is None:par=E.SubElement(pars,'parameter',name=name,type='double',unit='')
    par.set('value',str(val))
tree.write(p,encoding='utf-8',xml_declaration=True)
d=run(p,step='0,0.00005,0.5');x=d['Cilindro_1#x#Value'];pA=d['Cilindro_1#p_A#Value'];pB=d['Cilindro_1#p_B#Value']
invA=pA*(.00005+.001*x)**1.4;invB=pB*(.00005+.0008*(.1-x))**1.4
err=max(np.ptp(invA)/invA[0],np.ptp(invB)/invB[0]);assert err<3e-6 # CSV exports about seven significant digits.
res=max(abs(d['Cilindro_1#residuo_energia#Value']));assert res<1e-6
records.append({'circuito':'Teste adiabatico fechado','aprovado':True,'variacao_relativa_pVgamma':float(err),'residuo_energia_max_J':float(res)})
(out/'validacao_cilindro.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
print('Quatro circuitos, quatro comparacoes de passo e teste adiabatico aprovados.')
