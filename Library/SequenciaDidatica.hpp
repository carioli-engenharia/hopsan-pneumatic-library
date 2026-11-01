// SPDX-License-Identifier: Apache-2.0
#ifndef SEQUENCIA_DIDATICA_HPP
#define SEQUENCIA_DIDATICA_HPP
#include "ComponentEssentials.h"
namespace hopsan {
class SequenciaDidatica : public ComponentSignal {
    double *u;
public:
    static Component* Creator(){return new SequenciaDidatica();}
    void configure() override {addOutputVariable("u","0 then advance then centre then retract then centre","",0,&u);}
    void initialize() override {*u=0;}
    void simulateOneTimestep() override {*u=(mTime>=.2 && mTime<.32)?1:((mTime>=1 && mTime<1.12)?-1:0);}
};
}
#endif
