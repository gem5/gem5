#include "vortex-gpu/vortex_gpu.hh"
#include "debug/Vortex.hh"
#include "sim/system.hh"
#include "mem/request.hh"

namespace gem5
{

Vortex::Vortex(const VortexParams &p)
    : PioDevice(p),
    pioAddr(p.pio_addr),
    intGpu(p.int_gpu)
{
    DPRINTF(Vortex, "Creating Vortex\n");

    vx_device_h device = nullptr;
    vx_dev_open(&device);

    uint64_t num_cores;
    //vx_dev_caps(device, VX_CAPS_NUM_CORES, &num_cores);
    //DPRINTF(Vortex, "num cores = %x\n", VX_CAPS_NUM_CORES);
}

void
Vortex::init()
{
    PioDevice::init();

    // reset the GPU
    reset();
}

void Vortex::serialize(CheckpointOut &cp) const
{
}

void Vortex::unserialize(CheckpointIn &cp)
{
}

Tick Vortex::read(PacketPtr pkt) 
{
    DPRINTF(Vortex, "read()\n");

    // example read
    pkt->makeResponse();

    return 0;
}

Tick Vortex::write(PacketPtr pkt) 
{
    DPRINTF(Vortex, "write()\n");

    // example write
    pkt->makeResponse();

    return 0;
}

AddrRangeList Vortex::getAddrRanges() const
{
    return AddrRangeList({ RangeSize(pioAddr, pioAddr + 0xFFFF) });
}

void Vortex::reset() 
{
    DPRINTF(Vortex, "reset()\n");
}

} // namespace gem5