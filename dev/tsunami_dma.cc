/* $Id$ */

/* @file
 * Tsunami DMA fake
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "dev/console.hh"
#include "dev/etherdev.hh"
#include "dev/scsi_ctrl.hh"
#include "dev/tlaser_clock.hh"
#include "dev/tsunami_dma.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

TsunamiDMA::TsunamiDMA(const string &name, /*Tsunami *t,*/
                       Addr addr, Addr mask, MemoryController *mmu)
    : MmapDevice(name, addr, mask, mmu)/*, tsunami(t) */
{

}

Fault
TsunamiDMA::read(MemReqPtr req, uint8_t *data)
{
    DPRINTF(Tsunami, "dma read  va=%#x size=%d IOPorrt=%#x\n",
            req->vaddr, req->size, req->vaddr & 0xfff);

 //   Addr daddr = (req->paddr & addr_mask) >> 6;
//    ExecContext *xc = req->xc;
//    int cpuid = xc->cpu_id;
    *(uint64_t*)data = 0x00;

    return No_Fault;
}

Fault
TsunamiDMA::write(MemReqPtr req, const uint8_t *data)
{
    DPRINTF(Tsunami, "dma write - va=%#x size=%d IOPort=%#x\n",
            req->vaddr, req->size, req->vaddr & 0xfff);

    //Addr daddr = (req->paddr & addr_mask) >> 6;

    return No_Fault;
}

void
TsunamiDMA::serialize(std::ostream &os)
{
    // code should be written
}

void
TsunamiDMA::unserialize(Checkpoint *cp, const std::string &section)
{
    //code should be written
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiDMA)

 //   SimObjectParam<Tsunami *> tsunami;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiDMA)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiDMA)

//    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask")

END_INIT_SIM_OBJECT_PARAMS(TsunamiDMA)

CREATE_SIM_OBJECT(TsunamiDMA)
{
    return new TsunamiDMA(getInstanceName(), /*tsunami,*/ addr, mask, mmu);
}

REGISTER_SIM_OBJECT("TsunamiDMA", TsunamiDMA)
