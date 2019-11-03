#define SC_INCLUDE_DYNAMIC_PROCESSES
#include "CoreDecouplingLTInitiator.h"
#include "SimpleATInitiator1.h"
#include "SimpleATInitiator2.h"
#include "SimpleLTInitiator1.h"
#include "SimpleLTInitiator2.h"
#include "SimpleLTInitiator3.h"
#include "SimpleATTarget1.h"
#include "SimpleATTarget2.h"
#include "SimpleLTTarget1.h"
#include "SimpleLTTarget2.h"
#include "ExplicitATTarget.h"
#include "ExplicitLTTarget.h"

#include "MultiSocketSimpleSwitchAT.h"

/*
Connection Map: Busses are written vertically

AT_I1-B-AT_T1
LT_I1-U-LT_T1
LT_I2-S
LT_I3-1-B-LT_T2
        U-LT_T3
  AT_I2-S-AT_T2
  LT_I4-2-AT_T3
*/
int sc_main(int argc, char* argv[]){

    MultiSocketSimpleSwitchAT bus1("bus1");
    MultiSocketSimpleSwitchAT bus2("bus2");
    
    SimpleATInitiator1 at_i1("AT_I1", 3, 0x10000000); //AT_T1
    SimpleLTInitiator1 lt_i1("LT_I1", 3, 0x20000000); //LT_T1
    SimpleLTInitiator2 lt_i2("LT_I2", 3, 0x30000000); //LT_T2
    SimpleLTInitiator3 lt_i3("LT_I3", 3, 0x60000000); //AT_T3

    
    SimpleATInitiator2 at_i2("AT_I2", 3, 0x50000000); //AT_T2
    CoreDecouplingLTInitiator lt_i4("LT_I4", 3, 0x40000000); //LT_T3
    
    SimpleATTarget1    at_t1("AT_T1");
    SimpleATTarget2    at_t2("AT_T2");
    ExplicitATTarget    at_t3("AT_T3");
    SimpleLTTarget1    lt_t1("LT_T1");
    SimpleLTTarget2    lt_t2("LT_T2");
    ExplicitLTTarget    lt_t3("LT_T3");
    
    
    at_i1.socket(bus1.target_socket);
    lt_i1.socket(bus1.target_socket);
    lt_i2.socket(bus1.target_socket);
    lt_i3.socket(bus1.target_socket);

    at_i2.socket(bus2.target_socket);
    lt_i4.socket(bus2.target_socket);

    bus1.bindTargetSocket(at_t1.socket, 0x10000000,0x1000ffff, 0xfffffff);
    bus1.bindTargetSocket(lt_t1.socket, 0x20000000,0x2000ffff, 0xfffffff);
    bus1.bindTargetSocket(bus2.target_socket, 0x30000000,0x6000ffff);

    bus2.bindTargetSocket(at_t2.socket, 0x50000000,0x5000ffff, 0xfffffff);
    bus2.bindTargetSocket(lt_t2.socket, 0x30000000,0x3000ffff, 0xfffffff);
    bus2.bindTargetSocket(at_t3.socket, 0x60000000,0x6000ffff, 0xfffffff);
    bus2.bindTargetSocket(lt_t3.socket, 0x40000000,0x4000ffff, 0xfffffff);
    
    sc_core::sc_start();

    bus1.dump_status();
    bus2.dump_status();

    return 0;
}
