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
#include "dev/tsunami_io.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

TsunamiIO::TsunamiIO(const string &name, /*Tsunami *t,*/
                       Addr addr, Addr mask, MemoryController *mmu)
    : MmapDevice(name, addr, mask, mmu)/*, tsunami(t) */
{

}

Fault
TsunamiIO::read(MemReqPtr req, uint8_t *data)
{
    DPRINTF(Tsunami, "io read  va=%#x size=%d IOPorrt=%#x\n",
            req->vaddr, req->size, req->vaddr & 0xfff);

 //   Addr daddr = (req->paddr & addr_mask) >> 6;
//    ExecContext *xc = req->xc;
//    int cpuid = xc->cpu_id;
     panic("I/O Read - va%#x size %d\n", req->vaddr, req->size);
   // *(uint64_t*)data = 0x00;

    return No_Fault;
}

Fault
TsunamiIO::write(MemReqPtr req, const uint8_t *data)
{
    DPRINTF(Tsunami, "io write - va=%#x size=%d IOPort=%#x\n",
            req->vaddr, req->size, req->vaddr & 0xfff);

    Addr daddr = (req->paddr & addr_mask);

    switch(req->size) {
        case sizeof(uint8_t):
            switch(daddr) {
                case TSDEV_PIC1_MASK:
                    mask1 = *(uint8_t*)data;
                    return No_Fault;
                case TSDEV_PIC2_MASK:
                    mask2 = *(uint8_t*)data;
                    return No_Fault;
                case TSDEV_DMA1_RESET:
                    return No_Fault;
                case TSDEV_DMA2_RESET:
                    return No_Fault;
                case TSDEV_DMA1_MODE:
                    mode1 = *(uint8_t*)data;
                    return No_Fault;
                case TSDEV_DMA2_MODE:
                    mode2 = *(uint8_t*)data;
                    return No_Fault;
                case TSDEV_DMA1_MASK:
                case TSDEV_DMA2_MASK:
                    return No_Fault;
                default:
                    panic("I/O Write - va%#x size %d\n", req->vaddr, req->size);
            }
        case sizeof(uint16_t):
        case sizeof(uint32_t):
        case sizeof(uint64_t):
        default:
            panic("I/O Write - invalid size - va %#x size %d\n", req->vaddr, req->size);
    }


    return No_Fault;
}

void
TsunamiIO::serialize(std::ostream &os)
{
    // code should be written
}

void
TsunamiIO::unserialize(Checkpoint *cp, const std::string &section)
{
    //code should be written
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiIO)

 //   SimObjectParam<Tsunami *> tsunami;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiIO)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

//    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask")

END_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

CREATE_SIM_OBJECT(TsunamiIO)
{
    return new TsunamiIO(getInstanceName(), /*tsunami,*/ addr, mask, mmu);
}

REGISTER_SIM_OBJECT("TsunamiIO", TsunamiIO)
