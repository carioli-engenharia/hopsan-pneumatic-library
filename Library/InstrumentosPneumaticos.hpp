// SPDX-License-Identifier: Apache-2.0
#ifndef INSTRUMENTOS_PNEUMATICOS_HPP
#define INSTRUMENTOS_PNEUMATICOS_HPP
#include "ComponentEssentials.h"
#include <cmath>
namespace hopsan {
class InstrumentoPressao : public ComponentSignal {
    Port* port;double *p,*atm,*threshold,*hyst,*absOut,*barOut,*switchOut;bool state=false;
public:
    static Component* Creator(){return new InstrumentoPressao();}
    void configure()override{
        port=addReadPort("P","NodePneumatic");
        addInputVariable("P_atm","Ambient absolute pressure","Pa",100000,&atm);
        addInputVariable("P_ligar","Gauge pressure switch-on threshold","Pa",200000,&threshold);
        addInputVariable("Histerese","Switch-off differential","Pa",10000,&hyst);
        addOutputVariable("p_abs","Absolute pressure","Pa",0,&absOut);
        addOutputVariable("p_bar","Gauge pressure","bar",0,&barOut);
        addOutputVariable("ligado","Pressure switch state","",0,&switchOut);
    }
    void initialize()override{p=getSafeNodeDataPtr(port,NodePneumatic::Pressure);state=false;simulateOneTimestep();}
    void simulateOneTimestep()override{
        const double gauge=*p-*atm;if(gauge>=*threshold)state=true;else if(gauge<=*threshold-std::abs(*hyst))state=false;
        *absOut=*p;*barOut=gauge/1e5;*switchOut=state?1:0;
    }
};
class FinsCurso : public ComponentSignal {
    double *x,*length,*tol,*back,*front;
public:
    static Component* Creator(){return new FinsCurso();}
    void configure()override{
        addInputVariable("x","Cylinder extension","m",0,&x);addInputVariable("Curso","Cylinder stroke","m",.1,&length);addInputVariable("Tolerancia","Sensing distance from stroke end","m",.001,&tol);
        addOutputVariable("recuado","Retracted position detected","",0,&back);addOutputVariable("avancado","Extended position detected","",0,&front);
    }
    void initialize()override{simulateOneTimestep();}
    void simulateOneTimestep()override{*back=*x<=*tol?1:0;*front=*x>=*length-*tol?1:0;}
};
class ComandoDuasBobinas : public ComponentSignal {
    double *a,*b,*out,*fault;
public:
    static Component* Creator(){return new ComandoDuasBobinas();}
    void configure()override{
        addInputVariable("avancar","Extension coil request","",0,&a);addInputVariable("recuar","Retraction coil request","",0,&b);
        addOutputVariable("u","Directional valve command","",0,&out);addOutputVariable("conflito","Both requests active","",0,&fault);
    }
    void initialize()override{simulateOneTimestep();}
    void simulateOneTimestep()override{bool A=*a>.5,B=*b>.5;*out=(A&&!B)?1:((B&&!A)?-1:0);*fault=A&&B?1:0;}
};
class TemporizadorTON : public ComponentSignal {
    double *input,*delay,*out,*elapsed;double clock=0;
public:
    static Component* Creator(){return new TemporizadorTON();}
    void configure()override{addInputVariable("in","Enable","",0,&input);addInputVariable("Atraso","On-delay","s",.5,&delay);addOutputVariable("out","Delayed output","",0,&out);addOutputVariable("decorrido","Elapsed enabled time","s",0,&elapsed);}
    void initialize()override{clock=0;*out=0;*elapsed=0;}
    void simulateOneTimestep()override{if(*input>.5)clock+=mTimestep;else clock=0;*out=(*input>.5 && clock>=*delay)?1:0;*elapsed=clock;}
};
class ReservatorioAr : public ComponentC {
    Port*port;double *q,*e,*c,*z,*T,*P,*vol,*p0,*t0,*atmT,*ua,*pm,*mm,*tt;
    double m,U;
public:
    static Component* Creator(){return new ReservatorioAr();}
    void configure()override{
        port=addPowerPort("P","NodePneumatic");
        addInputVariable("Volume","Fixed gas volume","m3",.001,&vol);addInputVariable("p_inicial","Initial absolute pressure","Pa",100000,&p0);addInputVariable("T_inicial","Initial temperature","K",293,&t0);
        addInputVariable("T_ambiente","Ambient temperature","K",293,&atmT);addInputVariable("UA","Wall heat transfer conductance","W/K",0,&ua);
        addOutputVariable("p_abs","Absolute pressure","Pa",0,&pm);addOutputVariable("massa","Gas mass","kg",0,&mm);addOutputVariable("temperatura","Gas temperature","K",0,&tt);
    }
    void initialize()override{
        if(*vol<=0||*p0<=0||*t0<=0||*ua<0){stopSimulation("Reservoir: invalid parameters.");return;}
        q=getSafeNodeDataPtr(port,NodePneumatic::MassFlow);e=getSafeNodeDataPtr(port,NodePneumatic::EnergyFlow);c=getSafeNodeDataPtr(port,NodePneumatic::WaveVariable);z=getSafeNodeDataPtr(port,NodePneumatic::CharImpedance);T=getSafeNodeDataPtr(port,NodePneumatic::Temperature);P=getSafeNodeDataPtr(port,NodePneumatic::Pressure);
        m=*p0**vol/(287**t0);U=m*717.5**t0;*q=0;*e=0;*P=*p0;publish();
    }
    void publish(){*T=U/(m*717.5);*c=.4*U/ *vol;*z=0;*pm=*c;*mm=m;*tt=*T;}
    void simulateOneTimestep()override{
        m+=mTimestep**q;U+=mTimestep*(*e+*ua*(*atmT-*T));
        if(m<=0||U<=0||!std::isfinite(U)||!std::isfinite(m)){stopSimulation("Reservoir: invalid state; reduce timestep.");return;}publish();
    }
};
}
#endif
