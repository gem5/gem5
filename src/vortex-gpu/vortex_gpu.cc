#include "vortex-gpu/vortex_gpu.hh"
#include "debug/Vortex.hh"
#include "sim/system.hh"
#include "mem/request.hh"
#include "arch.h"
#include "processor.h"
#include "mem.h"
#include "constants.h"
#include <util.h>
#include "core.h"
#include "VX_types.h"

namespace gem5
{

Vortex::Vortex(const VortexParams &p)
    : PioDevice(p),
    pioAddr(p.pio_addr),
    intGpu(p.int_gpu),
    numClusters(p.num_clusters),
    numCores(p.num_cores),
    numWarps(p.num_warps),
    numThreads(p.num_threads),
    tickEvent([this]{processTick();}, name())
{
    DPRINTF(Vortex, "Creating Vortex\n");

    vortex::Arch arch(numThreads, numWarps, numCores);

    // create memory module
    // vortex::RAM ram(0, MEM_PAGE_SIZE);

    // create processor
    vortex::Processor processor(arch);

    DPRINTF(Vortex, "Created Processor\n");
    // attach memory module
    // processor.attach_ram(&ram);

	  // setup base DCRs
    const uint64_t startup_addr(STARTUP_ADDR);
    processor.dcr_write(VX_DCR_BASE_STARTUP_ADDR0, startup_addr & 0xffffffff);
  #if (XLEN == 64)
    processor.dcr_write(VX_DCR_BASE_STARTUP_ADDR1, startup_addr >> 32);
  #endif
	processor.dcr_write(VX_DCR_BASE_MPM_CLASS, 0);

    DPRINTF(Vortex, "DCR Setup\n");

    processor.run();

    DPRINTF(Vortex, "Vortex Running\n");

    // read exitcode from @MPM.1
    //ram.read(&exitcode, (IO_MPM_ADDR + 8), 4);
}

void
Vortex::init()
{
    PioDevice::init();
    schedule(tickEvent, 1);
}

void Vortex::processTick() 
{
    // FIX THIS SIMPLATFORM ISSUE
    SimPlatform::instance().tick();
    schedule(tickEvent, curTick() + 1);
}

void Vortex::serialize(CheckpointOut &cp) const
{
}

void Vortex::unserialize(CheckpointIn &cp)
{
}

Tick Vortex::read(PacketPtr pkt) 
{
    const Addr addr(pkt->getAddr() - pioAddr);
    uint32_t* value;
    vortex_read(device, addr, value);

    // example read
    pkt->setLE<uint32_t>(*value);
    pkt->makeResponse();

    return 0;
}

Tick Vortex::write(PacketPtr pkt) 
{
    const Addr addr(pkt->getAddr() - pioAddr);
    vortex_write(device, addr, pkt->getLE<uint32_t>());

    // example write
    pkt->makeAtomicResponse();

    return 0;
}

AddrRangeList Vortex::getAddrRanges() const
{
    return AddrRangeList({ RangeSize(pioAddr, pioAddr + 0xFFFF) });
}

int Vortex::vortex_read(vx_device_h hdevice, uint32_t addr, uint32_t* value) 
{
    DPRINTF(Vortex, "read()\n");

    vx_dcr_read(hdevice, addr, value);

    return 0;
}

int Vortex::vortex_write(vx_device_h hdevice, uint32_t addr, uint32_t value) 
{
    DPRINTF(Vortex, "write()\n");

    vx_dcr_write(hdevice, addr, value);

    return 0;
}

} // namespace gem5