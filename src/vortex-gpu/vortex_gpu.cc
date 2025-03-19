#include "vortex-gpu/vortex_gpu.hh"
#include "debug/Vortex.hh"
#include "sim/system.hh"
#include "mem/request.hh"

namespace gem5
{

Vortex::Vortex(const VortexParams &p)
    : PioDevice(p),
    pioAddr(p.pio_addr),
    intGpu(p.int_gpu),
    numClusters(p.num_clusters),
    numCores(p.num_cores),
    numWarps(p.num_warps),
    numThreads(p.num_threads)
{
    DPRINTF(Vortex, "Creating Vortex\n");

    // open device connection
    device = nullptr;
    vx_dev_open(&device);

    // checking core configuration
    uint64_t num_cores;
    vx_dev_caps(device, VX_CAPS_NUM_CORES, &num_cores);
    DPRINTF(Vortex, "num cores = %x\n", num_cores);
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
    const Addr addr(pkt->getAddr() - pioAddr);
    int* value;
    vortex_read(device, addr, value);

    // example read
    pkt->setLE<uint32_t>(value);
    pkt->makeResponse();

    return 0;
}

Tick Vortex::write(PacketPtr pkt) 
{
    DPRINTF(Vortex, "write()\n");

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