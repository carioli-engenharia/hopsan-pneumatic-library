// SPDX-License-Identifier: Apache-2.0
// Pneumatica Didatica. Autoria em AUTORIA.txt. 2026-11-01.
// Two adiabatic ideal-gas chambers, variable volume and integrated moving mass.
// C component: explicit pressure boundary, zero characteristic impedance.
// Requires timestep convergence checks. Not a certified industrial component.
#ifndef CILINDRO_DUPLA_ACAO_HPP
#define CILINDRO_DUPLA_ACAO_HPP
#include "ComponentEssentials.h"
#include <array>
#include <cmath>
#include <algorithm>
namespace hopsan {
template<bool Single=false> class CilindroDuplaAcao : public ComponentC {
    Port *ports[2];
    double *q[2],*e[2],*p[2],*t[2],*c[2],*z[2];
    double *areaA,*areaB,*length,*dead,*mass,*damping,*load,*atm,*initialX,*initialP,*initialT,*initialV;
    double *xo,*vo,*pa,*pb,*ma,*mb,*energy,*loss,*balance;
    double *spring,*preload;
    double dummy[6]={0,0,100000,293,100000,0};
    std::array<double,8> y; // x,v,mA,mB,UA,UB,integral external power,impact loss
    double startEnergy=0;
    static constexpr double R=287.0,cv=717.5;
    std::array<double,8> derivative(const std::array<double,8>& s) {
        std::array<double,8> d{};
        const double VA=*dead+*areaA*s[0],VB=*dead+*areaB*(*length-s[0]);
        const double pA=(R/cv)*s[4]/VA,pB=(R/cv)*s[5]/VB;
        double v=s[1];
        const double force=pA**areaA-pB**areaB-*atm*(*areaA-*areaB)-*load-*preload-*spring*s[0]-*damping*v;
        double a=force/ *mass;
        if((s[0]<=0 && v<=0 && a<0)||(s[0]>=*length && v>=0 && a>0)){v=0;a=0;}
        d[0]=v;d[1]=a;d[2]=*q[0];d[3]=*q[1];
        d[4]=*e[0]-pA**areaA*v;d[5]=*e[1]+pB**areaB*v;
        d[6]=*e[0]+*e[1]-(*atm*(*areaA-*areaB)+*load+*preload)*v-*damping*v*v;
        return d;
    }
    void publish() {
        const double VA=*dead+*areaA*y[0],VB=*dead+*areaB*(*length-y[0]);
        for(int i=0;i<2;++i){
            *t[i]=y[4+i]/(cv*y[2+i]);*c[i]=(R/cv)*y[4+i]/(i?VB:VA);*z[i]=0;
            // Pressure is written by the connected Q component during simulation.
        }
        *xo=y[0];*vo=y[1];*pa=*c[0];*pb=*c[1];*ma=y[2];*mb=y[3];
        *energy=y[4]+y[5]+.5**mass*y[1]*y[1]+.5**spring*y[0]*y[0];*loss=y[7];
        *balance=*energy-startEnergy-y[6]+y[7];
    }
public:
    static Component* Creator(){return new CilindroDuplaAcao<Single>();}
    void configure() override {
        ports[0]=addPowerPort("A","NodePneumatic");if(!Single)ports[1]=addPowerPort("B","NodePneumatic");
        addInputVariable("Area_A","Full piston area","m2",.001,&areaA);
        addInputVariable("Area_B","Rod side annular area","m2",.0008,&areaB);
        addInputVariable("Curso","Stroke","m",.1,&length);
        addInputVariable("Volume_morto","Dead volume per chamber","m3",.00005,&dead);
        addInputVariable("Massa","Piston plus rigidly attached load","kg",2,&mass);
        addInputVariable("B_viscoso","Viscous mechanical damping","Ns/m",120,&damping);
        addInputVariable("Carga","Constant force opposing extension","N",0,&load);
        addInputVariable("K_mola","Return spring stiffness","N/m",Single?1500:0,&spring);
        addInputVariable("Pre_carga","Constant return spring preload","N",Single?20:0,&preload);
        addInputVariable("P_atm","Ambient absolute pressure on rod","Pa",100000,&atm);
        addInputVariable("x_inicial","Initial extension","m",.05,&initialX);
        addInputVariable("v_inicial","Initial velocity","m/s",0,&initialV);
        addInputVariable("p_inicial","Initial absolute chamber pressure","Pa",100000,&initialP);
        addInputVariable("T_inicial","Initial gas temperature","K",293,&initialT);
        addOutputVariable("x","Extension","m",0,&xo);addOutputVariable("v","Velocity","m/s",0,&vo);
        addOutputVariable("p_A","Absolute chamber A pressure","Pa",0,&pa);addOutputVariable("p_B","Absolute chamber B pressure","Pa",0,&pb);
        addOutputVariable("massa_A","Chamber A mass","kg",0,&ma);addOutputVariable("massa_B","Chamber B mass","kg",0,&mb);
        addOutputVariable("energia","Gas internal plus kinetic energy","J",0,&energy);
        addOutputVariable("perda_batente","Cumulative inelastic impact loss","J",0,&loss);
        addOutputVariable("residuo_energia","Energy accounting residual","J",0,&balance);
    }
    void initialize() override {
        if(Single)*areaB=0;
        if(!(*areaA>0 && (Single || *areaB>0) && *areaB<=*areaA && *length>0 && *dead>0 && *mass>0 && *damping>=0 && *spring>=0 && *preload>=0 && *atm>0 && *initialP>0 && *initialT>0 && *initialX>=0 && *initialX<=*length)){
            stopSimulation("Cylinder: invalid physical parameters.");return;
        }
        for(int i=0;i<2;++i){
            if(Single && i==1){q[i]=&dummy[0];e[i]=&dummy[1];p[i]=&dummy[2];t[i]=&dummy[3];c[i]=&dummy[4];z[i]=&dummy[5];continue;}
            q[i]=getSafeNodeDataPtr(ports[i],NodePneumatic::MassFlow);e[i]=getSafeNodeDataPtr(ports[i],NodePneumatic::EnergyFlow);
            p[i]=getSafeNodeDataPtr(ports[i],NodePneumatic::Pressure);t[i]=getSafeNodeDataPtr(ports[i],NodePneumatic::Temperature);
            c[i]=getSafeNodeDataPtr(ports[i],NodePneumatic::WaveVariable);z[i]=getSafeNodeDataPtr(ports[i],NodePneumatic::CharImpedance);
            *q[i]=0;*e[i]=0;*p[i]=*initialP;
        }
        y={*initialX,*initialV,0,0,0,0,0,0};
        y[2]=*initialP*(*dead+*areaA*y[0])/(R**initialT);
        y[3]=*initialP*(*dead+*areaB*(*length-y[0]))/(R**initialT);
        y[4]=y[2]*cv**initialT;y[5]=y[3]*cv**initialT;
        startEnergy=y[4]+y[5]+.5**mass*y[1]*y[1]+.5**spring*y[0]*y[0];publish();
    }
    void simulateOneTimestep() override {
        auto k1=derivative(y),temp=y;
        for(int i=0;i<8;++i)temp[i]=y[i]+.5*mTimestep*k1[i];
        auto k2=derivative(temp);
        for(int i=0;i<8;++i)temp[i]=y[i]+.5*mTimestep*k2[i];
        auto k3=derivative(temp);
        for(int i=0;i<8;++i)temp[i]=y[i]+mTimestep*k3[i];
        auto k4=derivative(temp);
        for(int i=0;i<8;++i)y[i]+=mTimestep*(k1[i]+2*k2[i]+2*k3[i]+k4[i])/6;
        if(y[0]<0 || y[0]>*length){
            // Correct gas energies for the tiny overshoot using p V^gamma=const.
            const double oldVA=*dead+*areaA*y[0],oldVB=*dead+*areaB*(*length-y[0]);
            const double x=std::max(0.0,std::min(*length,y[0]));
            const double newUA=y[4]*std::pow(oldVA/(*dead+*areaA*x),.4);
            const double newUB=y[5]*std::pow(oldVB/(*dead+*areaB*(*length-x)),.4);
            y[7]+=.5**mass*y[1]*y[1];
            // Clamp correction contributes to the numerical energy residual.
            y[0]=x;y[1]=0;y[4]=newUA;y[5]=newUB;
        }
        for(double value:y)if(!std::isfinite(value)){stopSimulation("Cylinder: non-finite state; reduce timestep.");return;}
        if(y[2]<=0 || y[3]<=0 || y[4]<=0 || y[5]<=0){stopSimulation("Cylinder: non-positive gas state; reduce timestep or flow area.");return;}
        publish();
    }
};
}
#endif
