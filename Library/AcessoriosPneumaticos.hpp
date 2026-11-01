// SPDX-License-Identifier: Apache-2.0
// Pneumatica Didatica; autoria em AUTORIA.txt, 2026-11-01.
// Interface: Hopsan 2.24.4; API copyright Hopsan Group, Apache-2.0.
// Ideal directional valves: compressible air, quasi-steady orifices,
// upstream enthalpy transport, no spool dynamics or closed-port leakage.
#ifndef ACESSORIOS_PNEUMATICOS_HPP
#define ACESSORIOS_PNEUMATICOS_HPP
#include "ComponentEssentials.h"
#include <algorithm>
#include <cmath>
#include <array>
#include <limits>
namespace hopsan {
// Modes 10 throttle, 11 check, 12 one-way throttle, 13 OR, 14 AND,
// 15 quick exhaust, 16 relieving regulator, 17 relief, 18 shutoff.
template<int Mode> class PneumaticAccessory : public ComponentQ {
    static constexpr int N = (Mode>=13 && Mode<=16) ? 3:2;
    Port* port[N];
    double *np[N], *nq[N], *ne[N], *nt[N], *nc[N], *nz[N];
    double *u,*area,*cd,*stateOut,*resOut,*massOut,*energyOut;
    double *reverseArea,*crack,*setpoint,*band;
    double c[N],z[N],T[N],p[N],opening[4];
    int state=-1;
    // Port order: P(1), A(2), R(3), B(4), S(5).
    static double flow(double p1,double p2,double t1,double t2,double a,double C) {
        if(a==0 || p1==p2) return 0;
        const double hi=std::max(p1,p2),lo=std::min(p1,p2);
        const double temp=p1>p2?t1:t2, ratio=lo/hi, g=1.4,R=287.0;
        const double critical=std::pow(2/(g+1),g/(g-1));
        double factor;
        if(ratio<=critical) factor=std::sqrt(g/R)*std::pow(2/(g+1),(g+1)/(2*(g-1)));
        else factor=std::sqrt(std::max(0.0,2*g/(R*(g-1))*(std::pow(ratio,2/g)-std::pow(ratio,(g+1)/g))));
        double mass=C*a*hi/std::sqrt(temp)*factor;
        // Smooth the square-root cusp only below 1 Pa pressure difference.
        const double dp=hi-lo;
        if(dp<1.0) mass*=std::sqrt(dp);
        return p1>p2?mass:-mass;
    }
    void evaluate(const double* trial,double* f,double* q,double* e) {
        std::fill(q,q+N,0);std::fill(e,e+N,0);
        auto edge=[&](int i,int j,double a,bool oneWay=false,double drop=0.0){
            if(a<=0 || (oneWay && trial[i]<=trial[j]+drop))return;
            double m=flow(trial[i]-(oneWay?drop:0),trial[j],T[i],T[j],a,*cd);
            double h=1004.5*(m>=0?T[i]:T[j]);
            q[i]-=m;q[j]+=m;e[i]-=h*m;e[j]+=h*m;
        };
        auto unit=[](double value){return std::max(0.0,std::min(1.0,value));};
        if(Mode==10)edge(0,1,*area*unit(*u)); // adjustable two-way throttle
        if(Mode==11)edge(0,1,*area,true,*crack); // check valve
        if(Mode==12)edge(0,1,trial[0]>=trial[1]?*area:*reverseArea); // one-way flow control
        if(Mode==13 || Mode==14)edge(state==1?0:1,2,*area); // OR / two pressure
        if(Mode==15){if(state==1)edge(0,1,*area,true);else edge(1,2,*area,true);} // quick exhaust
        if(Mode==16){
            edge(0,1,*area*unit((*setpoint-(trial[1]-trial[2]))/ *band),true);
            edge(1,2,*area*unit(((trial[1]-trial[2])-*setpoint)/ *band),true);
        }
        if(Mode==17)edge(0,1,*area*unit(((trial[0]-trial[1])-*setpoint)/ *band),true);
        if(Mode==18)edge(0,1,*area*unit(*u)); // isolation with continuous command
        for(int i=0;i<N;++i) f[i]=trial[i]-c[i]-z[i]*e[i];
    }
    static double norm(const double* f) {
        double r=0;for(int i=0;i<N;++i){if(!std::isfinite(f[i]))return std::numeric_limits<double>::infinity();r=std::max(r,std::abs(f[i]));}return r;
    }
public:
    static Component* Creator(){return new PneumaticAccessory<Mode>();}
    void configure() override {
        const char* ordinary[3]={"P","A","R"};const char* logic[3]={"X","Y","A"};
        for(int i=0;i<N;++i)port[i]=addPowerPort((Mode==13||Mode==14)?logic[i]:ordinary[i],"NodePneumatic");
        addInputVariable("u","Opening fraction for throttle/isolation","",1,&u);
        addInputVariable("A_passagem","Effective passage area","m2",2e-6,&area);
        addInputVariable("Cd","Discharge coefficient","",.65,&cd);
        addInputVariable("A_retorno","Reverse free flow effective area","m2",8e-6,&reverseArea);
        addInputVariable("P_abertura","Check valve cracking differential","Pa",1000,&crack);
        addInputVariable("P_ajuste","Regulated/relief pressure above exhaust","Pa",300000,&setpoint);
        addInputVariable("Banda","Proportional regulation pressure band","Pa",10000,&band);
        addOutputVariable("estado","Selected position: -1, 0 or +1","",0,&stateOut);
        addOutputVariable("residuo","Maximum pneumatic TLM residual","Pa",0,&resOut);
        addOutputVariable("balanco_massa","Sum of port mass flows","kg/s",0,&massOut);
        addOutputVariable("balanco_energia","Sum of port energy flows","W",0,&energyOut);
    }
    void initialize() override {
        for(int i=0;i<N;++i){
            np[i]=getSafeNodeDataPtr(port[i],NodePneumatic::Pressure);
            nq[i]=getSafeNodeDataPtr(port[i],NodePneumatic::MassFlow);
            ne[i]=getSafeNodeDataPtr(port[i],NodePneumatic::EnergyFlow);
            nt[i]=getSafeNodeDataPtr(port[i],NodePneumatic::Temperature);
            nc[i]=getSafeNodeDataPtr(port[i],NodePneumatic::WaveVariable);
            nz[i]=getSafeNodeDataPtr(port[i],NodePneumatic::CharImpedance);
            p[i]=std::max(1.0,*np[i]);
        }
        state=-1;
    }
    void simulateOneTimestep() override {
        if(!std::isfinite(*u)||!std::isfinite(*area)||!std::isfinite(*cd)||*area<0||*cd<=0||*cd>1){
            stopSimulation("Pneumatic valve: invalid command, area or Cd.");return;
        }
        if(!std::isfinite(*reverseArea)||!std::isfinite(*crack)||!std::isfinite(*setpoint)||!std::isfinite(*band)||*reverseArea<0||*crack<0||*setpoint<0||*band<=0){stopSimulation("Pneumatic accessory: invalid parameter.");return;}
        for(int i=0;i<N;++i){
            c[i]=*nc[i];z[i]=*nz[i];T[i]=*nt[i];
            if(!std::isfinite(c[i])||!std::isfinite(z[i])||!std::isfinite(T[i])||z[i]<0||T[i]<=0){
                stopSimulation("Pneumatic valve: invalid pneumatic boundary.");return;
            }
            p[i]=std::max(1.0,c[i]);
        }
        state=0;
        if(Mode==13)state=c[0]>=c[1]?1:-1;
        if(Mode==14)state=c[0]<=c[1]?1:-1;
        if(Mode==15)state=c[0]>=c[1]-500?1:-1;
        double f[N],q[N],e[N];evaluate(p,f,q,e);
        // Damped Newton with numerical Jacobian and partial pivoting.
        // Abort on nonconvergence instead of silently using an invalid flow.
        for(int it=0;it<35 && norm(f)>1e-4;++it){
            double J[N][N+1];
            for(int j=0;j<N;++j){
                double t[N],ff[N],qq[N],ee[N];std::copy(p,p+N,t);
                double delta=std::max(1e-3,std::abs(p[j])*1e-7);t[j]+=delta;evaluate(t,ff,qq,ee);
                for(int i=0;i<N;++i)J[i][j]=(ff[i]-f[i])/delta;
            }
            for(int i=0;i<N;++i)J[i][N]=-f[i];
            for(int k=0;k<N;++k){
                int pivot=k;for(int i=k+1;i<N;++i)if(std::abs(J[i][k])>std::abs(J[pivot][k]))pivot=i;
                for(int j=k;j<=N;++j)std::swap(J[k][j],J[pivot][j]);
                if(std::abs(J[k][k])<1e-14){stopSimulation("Pneumatic valve: singular solver.");return;}
                for(int i=k+1;i<N;++i){double d=J[i][k]/J[k][k];for(int j=k;j<=N;++j)J[i][j]-=d*J[k][j];}
            }
            double step[N];for(int i=N-1;i>=0;--i){double v=J[i][N];for(int j=i+1;j<N;++j)v-=J[i][j]*step[j];step[i]=v/J[i][i];}
            bool accepted=false;
            for(double alpha=1;alpha>1e-7;alpha*=0.5){
                double t[N],ff[N],qq[N],ee[N];bool positive=true;
                for(int i=0;i<N;++i){t[i]=p[i]+alpha*step[i];positive=positive&&std::isfinite(t[i])&&t[i]>0;}
                if(!positive)continue;
                evaluate(t,ff,qq,ee);
                if(norm(ff)<norm(f)){std::copy(t,t+N,p);std::copy(ff,ff+N,f);accepted=true;break;}
            }
            if(!accepted)break;
        }
        evaluate(p,f,q,e);
        if(!std::isfinite(norm(f))||norm(f)>1e-3){stopSimulation("Pneumatic valve: solver did not converge; reduce timestep/check boundaries.");return;}
        *stateOut=state;*resOut=norm(f);*massOut=0;*energyOut=0;
        for(int i=0;i<N;++i){*np[i]=p[i];*nq[i]=q[i];*ne[i]=e[i];*massOut+=q[i];*energyOut+=e[i];}
    }
};
}
#endif

