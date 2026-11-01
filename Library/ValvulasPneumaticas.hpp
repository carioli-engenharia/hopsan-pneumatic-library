// SPDX-License-Identifier: Apache-2.0
// Pneumatica Didatica; autoria em AUTORIA.txt, 2026-11-01.
// Interface: Hopsan 2.24.4; API copyright Hopsan Group, Apache-2.0.
// Ideal directional valves: compressible air, quasi-steady orifices,
// upstream enthalpy transport, no spool dynamics or closed-port leakage.
#ifndef VALVULAS_PNEUMATICAS_HPP
#define VALVULAS_PNEUMATICAS_HPP
#include "ComponentEssentials.h"
#include <algorithm>
#include <cmath>
#include <array>
#include <limits>
namespace hopsan {
// Modes: 0=32NC, 1=32NO, 2=52spring, 3=53closed,
// 4=53exhaust, 5=53pressure, 6=52memory.
template<int Mode> class PneumaticDirectionalValve : public ComponentQ {
    static constexpr int N = Mode<2 ? 3:5;
    Port* port[N];
    double *np[N], *nq[N], *ne[N], *nt[N], *nc[N], *nz[N];
    double *u,*area,*cd,*stateOut,*resOut,*massOut,*energyOut;
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
        const int from[4]={0,1,0,3},to[4]={1,2,3,4};
        for(int k=0;k<(N==3?2:4);++k) {
            int i=from[k],j=to[k];
            double m=flow(trial[i],trial[j],T[i],T[j],*area*opening[k],*cd);
            double h=1004.5*(m>=0?T[i]:T[j]); // cp=kappa*R/(kappa-1)
            q[i]-=m;q[j]+=m;e[i]-=h*m;e[j]+=h*m;
        }
        for(int i=0;i<N;++i) f[i]=trial[i]-c[i]-z[i]*e[i];
    }
    static double norm(const double* f) {
        double r=0;for(int i=0;i<N;++i){if(!std::isfinite(f[i]))return std::numeric_limits<double>::infinity();r=std::max(r,std::abs(f[i]));}return r;
    }
public:
    static Component* Creator(){return new PneumaticDirectionalValve<Mode>();}
    void configure() override {
        const char* names[5]={"P","A","R","B","S"};
        for(int i=0;i<N;++i)port[i]=addPowerPort(names[i],"NodePneumatic");
        addInputVariable("u","Command: 0/1 for spring 2-position, -1/0/+1 for 3-position or memory","",0,&u);
        addInputVariable("A_passagem","Effective open passage area","m2",8e-6,&area);
        addInputVariable("Cd","Discharge coefficient","",0.65,&cd);
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
        if(Mode<3)state=*u>=0.5?1:-1;
        else if(Mode==6){if(*u>=0.5)state=1;else if(*u<=-0.5)state=-1;}
        else state=*u>=0.5?1:(*u<=-0.5?-1:0);
        std::fill(opening,opening+4,0);
        if(Mode<2){bool supply=(state==1);if(Mode==1)supply=!supply;opening[supply?0:1]=1;}
        else if(state==1){opening[0]=opening[3]=1;}
        else if(state==-1){opening[1]=opening[2]=1;}
        else if(Mode==4){opening[1]=opening[3]=1;}
        else if(Mode==5){opening[0]=opening[2]=1;}
        for(int i=0;i<N;++i){
            c[i]=*nc[i];z[i]=*nz[i];T[i]=*nt[i];
            if(!std::isfinite(c[i])||!std::isfinite(z[i])||!std::isfinite(T[i])||z[i]<0||T[i]<=0){
                stopSimulation("Pneumatic valve: invalid pneumatic boundary.");return;
            }
            p[i]=std::max(1.0,c[i]);
        }
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

