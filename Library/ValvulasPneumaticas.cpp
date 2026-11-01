// SPDX-License-Identifier: Apache-2.0
#include "ValvulasPneumaticas.hpp"
#include "CilindroDuplaAcao.hpp"
#include "SequenciaDidatica.hpp"
#include "AcessoriosPneumaticos.hpp"
#include "InstrumentosPneumaticos.hpp"
using namespace hopsan;
extern "C" DLLEXPORT void register_contents(ComponentFactory* f, NodeFactory*) {
    f->registerCreatorFunction("PD_CilindroDuplaAcao",CilindroDuplaAcao<false>::Creator);
    f->registerCreatorFunction("PD_CilindroSimplesAcao",CilindroDuplaAcao<true>::Creator);
    f->registerCreatorFunction("PD_Estranguladora",PneumaticAccessory<10>::Creator);
    f->registerCreatorFunction("PD_Retencao",PneumaticAccessory<11>::Creator);
    f->registerCreatorFunction("PD_ReguladoraUnidirecional",PneumaticAccessory<12>::Creator);
    f->registerCreatorFunction("PD_SeletoraOU",PneumaticAccessory<13>::Creator);
    f->registerCreatorFunction("PD_DuasPressoesE",PneumaticAccessory<14>::Creator);
    f->registerCreatorFunction("PD_EscapeRapido",PneumaticAccessory<15>::Creator);
    f->registerCreatorFunction("PD_ReguladorPressao",PneumaticAccessory<16>::Creator);
    f->registerCreatorFunction("PD_Alivio",PneumaticAccessory<17>::Creator);
    f->registerCreatorFunction("PD_Bloqueio",PneumaticAccessory<18>::Creator);
    f->registerCreatorFunction("PD_ManometroPressostato",InstrumentoPressao::Creator);
    f->registerCreatorFunction("PD_FinsCurso",FinsCurso::Creator);
    f->registerCreatorFunction("PD_ComandoBobinas",ComandoDuasBobinas::Creator);
    f->registerCreatorFunction("PD_TemporizadorTON",TemporizadorTON::Creator);
    f->registerCreatorFunction("PD_Reservatorio",ReservatorioAr::Creator);
    f->registerCreatorFunction("PD_SequenciaDidatica",SequenciaDidatica::Creator);
    f->registerCreatorFunction("CPC_Valvula32NF",PneumaticDirectionalValve<0>::Creator);
    f->registerCreatorFunction("CPC_Valvula32NA",PneumaticDirectionalValve<1>::Creator);
    f->registerCreatorFunction("CPC_Valvula52Mola",PneumaticDirectionalValve<2>::Creator);
    f->registerCreatorFunction("CPC_Valvula53Fechada",PneumaticDirectionalValve<3>::Creator);
    f->registerCreatorFunction("CPC_Valvula53Exaustao",PneumaticDirectionalValve<4>::Creator);
    f->registerCreatorFunction("CPC_Valvula53Pressao",PneumaticDirectionalValve<5>::Creator);
    f->registerCreatorFunction("CPC_Valvula52Memoria",PneumaticDirectionalValve<6>::Creator);
}
extern "C" DLLEXPORT void get_hopsan_info(HopsanExternalLibInfoT *info) {
    info->libName=(char*)"PneumaticaDidatica";
    info->hopsanCoreVersion=(char*)HOPSANCOREVERSION;
    info->libCompiledDebugRelease=(char*)HOPSAN_BUILD_TYPE_STR;
}

