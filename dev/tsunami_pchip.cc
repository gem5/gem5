/* $Id$ */

/* @file
 * Tsunami PChip (pci)
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
#include "dev/tsunami_pchip.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

TsunamiPChip::TsunamiPChip(const string &name, Tsunami *t,
                       Addr addr, Addr mask, MemoryController *mmu)
    : MmapDevice(name, addr, mask, mmu), tsunami(t)
{
    wsba0 = 0;
    wsba1 = 0;
    wsba2 = 0;
    wsba3 = 0;
    wsm0 = 0;
    wsm1 = 0;
    wsm2 = 0;
    wsm3 = 0;
    tba0 = 0;
    tba1 = 0;
    tba2 = 0;
    tba3 = 0;

    //Set back pointer in tsunami
    tsunami->pchip = this;
}

Fault
TsunamiPChip::read(MemReqPtr req, uint8_t *data)
{
    DPRINTF(Tsunami, "read  va=%#x size=%d\n",
            req->vaddr, req->size);

    Addr daddr = (req->paddr & addr_mask) >> 6;
//    ExecContext *xc = req->xc;
//    int cpuid = xc->cpu_id;

    switch (req->size) {

      case sizeof(uint64_t):
          switch(daddr) {
              case TSDEV_PC_WSBA0:
                    *(uint64_t*)data = wsba0;
                    return No_Fault;
              case TSDEV_PC_WSBA1:
                    *(uint64_t*)data = wsba1;
                    return No_Fault;
              case TSDEV_PC_WSBA2:
                    *(uint64_t*)data = wsba2;
                    return No_Fault;
              case TSDEV_PC_WSBA3:
                    *(uint64_t*)data = wsba3;
                    return No_Fault;
              case TSDEV_PC_WSM0:
                    *(uint64_t*)data = wsm0;
                    return No_Fault;
              case TSDEV_PC_WSM1:
                    *(uint64_t*)data = wsm1;
                    return No_Fault;
              case TSDEV_PC_WSM2:
                    *(uint64_t*)data = wsm2;
                    return No_Fault;
              case TSDEV_PC_WSM3:
                    *(uint64_t*)data = wsm3;
                    return No_Fault;
              case TSDEV_PC_TBA0:
                    *(uint64_t*)data = tba0;
                    return No_Fault;
              case TSDEV_PC_TBA1:
                    *(uint64_t*)data = tba1;
                    return No_Fault;
              case TSDEV_PC_TBA2:
                    *(uint64_t*)data = tba2;
                    return No_Fault;
              case TSDEV_PC_TBA3:
                    *(uint64_t*)data = tba3;
                    return No_Fault;
              case TSDEV_PC_PCTL:
                    // might want to change the clock??
                    *(uint64_t*)data = 0x00; // try this
                    return No_Fault;
              case TSDEV_PC_PLAT:
                    panic("PC_PLAT not implemented\n");
              case TSDEV_PC_RES:
                    panic("PC_RES not implemented\n");
              case TSDEV_PC_PERROR:
                    panic("PC_PERROR not implemented\n");
              case TSDEV_PC_PERRMASK:
                    panic("PC_PERRMASK not implemented\n");
              case TSDEV_PC_PERRSET:
                    panic("PC_PERRSET not implemented\n");
              case TSDEV_PC_TLBIV:
                    panic("PC_TLBIV not implemented\n");
              case TSDEV_PC_TLBIA:
                    *(uint64_t*)data = 0x00; // shouldn't be readable, but linux
                    return No_Fault;
              case TSDEV_PC_PMONCTL:
                    panic("PC_PMONCTL not implemented\n");
              case TSDEV_PC_PMONCNT:
                    panic("PC_PMONCTN not implemented\n");
              default:
                  panic("Default in PChip Read reached reading 0x%x\n", daddr);

           } // uint64_t

      break;
      case sizeof(uint32_t):
      case sizeof(uint16_t):
      case sizeof(uint8_t):
      default:
        panic("invalid access size(?) for tsunami register!\n\n");
    }
    DPRINTFN("Tsunami PChip ERROR: read  daddr=%#x size=%d\n", daddr, req->size);

    return No_Fault;
}

Fault
TsunamiPChip::write(MemReqPtr req, const uint8_t *data)
{
    DPRINTF(Tsunami, "write - va=%#x size=%d \n",
            req->vaddr, req->size);

    Addr daddr = (req->paddr & addr_mask) >> 6;

    switch (req->size) {

      case sizeof(uint64_t):
          switch(daddr) {
              case TSDEV_PC_WSBA0:
                    wsba0 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSBA1:
                    wsba1 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSBA2:
                    wsba2 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSBA3:
                    wsba3 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM0:
                    wsm0 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM1:
                    wsm1 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM2:
                    wsm2 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_WSM3:
                    wsm3 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA0:
                    tba0 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA1:
                    tba1 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA2:
                    tba2 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_TBA3:
                    tba3 = *(uint64_t*)data;
                    return No_Fault;
              case TSDEV_PC_PCTL:
                    // might want to change the clock??
                    //*(uint64_t*)data; // try this
                    return No_Fault;
              case TSDEV_PC_PLAT:
                    panic("PC_PLAT not implemented\n");
              case TSDEV_PC_RES:
                    panic("PC_RES not implemented\n");
              case TSDEV_PC_PERROR:
                    panic("PC_PERROR not implemented\n");
              case TSDEV_PC_PERRMASK:
                    panic("PC_PERRMASK not implemented\n");
              case TSDEV_PC_PERRSET:
                    panic("PC_PERRSET not implemented\n");
              case TSDEV_PC_TLBIV:
                    panic("PC_TLBIV not implemented\n");
              case TSDEV_PC_TLBIA:
                    return No_Fault; // value ignored, supposted to invalidate SG TLB
              case TSDEV_PC_PMONCTL:
                    panic("PC_PMONCTL not implemented\n");
              case TSDEV_PC_PMONCNT:
                    panic("PC_PMONCTN not implemented\n");
              default:
                  panic("Default in PChip Read reached reading 0x%x\n", daddr);

           } // uint64_t

      break;
      case sizeof(uint32_t):
      case sizeof(uint16_t):
      case sizeof(uint8_t):
      default:
        panic("invalid access size(?) for tsunami register!\n\n");
    }

    DPRINTFN("Tsunami ERROR: write daddr=%#x size=%d\n", daddr, req->size);

    return No_Fault;
}

void
TsunamiPChip::serialize(std::ostream &os)
{
    // code should be written
}

void
TsunamiPChip::unserialize(Checkpoint *cp, const std::string &section)
{
    //code should be written
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiPChip)

    SimObjectParam<Tsunami *> tsunami;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiPChip)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiPChip)

    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask")

END_INIT_SIM_OBJECT_PARAMS(TsunamiPChip)

CREATE_SIM_OBJECT(TsunamiPChip)
{
    return new TsunamiPChip(getInstanceName(), tsunami, addr, mask, mmu);
}

REGISTER_SIM_OBJECT("TsunamiPChip", TsunamiPChip)
