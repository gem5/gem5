#ifndef __VORTEX_GPU_VORTEX_GPU_HH__
#define __VORTEX_GPU_VORTEX_GPU_HH__

#include "dev/io_device.hh"
#include "params/Vortex.hh"
#include "sim/eventq.hh"
#include "sim/system.hh"
#include "enums/MemoryMode.hh"
#include "mem/packet_access.hh"
#include "vortex/runtime/include/vortex.h"
#include "vortex/build/hw/VX_config.h"

namespace gem5
{
struct VortexParams;

class Vortex : public PioDevice
{
    public:
        Vortex(const VortexParams &p);

        void init() override;

    // Checkpointing
    // TODO: Implement this later
    public:
        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    
    public:
        Tick read(PacketPtr pkt) override;
        Tick write(PacketPtr pkt) override;
        AddrRangeList getAddrRanges() const override;

    private:
        EventFunctionWrapper tickEvent;

        void processTick();

    // Put wrapper functions here
    protected:
        int vortex_read(vx_device_h hdevice, uint32_t addr, uint32_t* value);
        int vortex_write(vx_device_h hdevice, uint32_t addr, uint32_t value);
    /*
    private:
        void setCallback(const_vortex_t &callback);
    };
    */

    protected:
        const Addr pioAddr;
        const uint32_t intGpu;
        const uint32_t numClusters;
        const uint32_t numCores;
        const uint32_t numWarps;
        const uint32_t numThreads;

        vx_device_h device;
};

} // namespace gem5

#endif // __VORTEX-GPU_VORTEX_GPU_HH__