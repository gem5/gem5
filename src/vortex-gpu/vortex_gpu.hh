#ifndef __VORTEX_GPU_VORTEX_GPU_HH__
#define __VORTEX_GPU_VORTEX_GPU_HH__

#include "dev/io_device.hh"
#include "params/Vortex.hh"
#include "sim/eventq.hh"
#include "sim/system.hh"
#include "runtime/include/vortex.h"


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

    // Put wrapper functions here
    protected:
        void reset();

    /*
    private:
        void setCallback(const_vortex_t &callback);
    };
    */

    protected:
        const Addr pioAddr;
        const uint32_t intGpu;
};

} // namespace gem5

#endif // __VORTEX-GPU_VORTEX_GPU_HH__