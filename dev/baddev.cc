/* $Id$ */

/* @file
 * BadDevice implemenation
 */

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "dev/scsi_ctrl.hh"
#include "dev/baddev.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "sim/system.hh"

using namespace std;

BadDevice::BadDevice(const string &name,
                       Addr addr, Addr mask, MemoryController *mmu, const string &devicename)
    : MmapDevice(name, addr, mask, mmu), devname(devicename)
{
}

Fault
BadDevice::read(MemReqPtr &req, uint8_t *data)
{

    panic("Device %s not imlpmented\n", devname);
    return No_Fault;
}

Fault
BadDevice::write(MemReqPtr &req, const uint8_t *data)
{
    panic("Device %s not imlpmented\n", devname);
    return No_Fault;
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(BadDevice)

    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;
    Param<string> devicename;

END_DECLARE_SIM_OBJECT_PARAMS(BadDevice)

BEGIN_INIT_SIM_OBJECT_PARAMS(BadDevice)

    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask"),
    INIT_PARAM(devicename, "Name of device to error on")

END_INIT_SIM_OBJECT_PARAMS(BadDevice)

CREATE_SIM_OBJECT(BadDevice)
{
    return new BadDevice(getInstanceName(), addr, mask, mmu, devicename);
}

REGISTER_SIM_OBJECT("BadDevice", BadDevice)
