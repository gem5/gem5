/* $Id$ */

/* @file
 * Tsunami I/O including PIC, PIT, RTC, DMA
 */

#include <sys/time.h>

#include <deque>
#include <string>
#include <vector>

#include "base/trace.hh"
#include "cpu/exec_context.hh"
#include "dev/console.hh"
#include "dev/tlaser_clock.hh"
#include "dev/tsunami_io.hh"
#include "dev/tsunamireg.h"
#include "dev/tsunami.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "dev/tsunami_cchip.hh"

using namespace std;

#define UNIX_YEAR_OFFSET 52

//This will have to be dynamic if we want support usermode access of the RTC
#define RTC_RATE  1024

// Timer Event for Periodic interrupt of RTC
TsunamiIO::RTCEvent::RTCEvent(Tsunami* t)
    : Event(&mainEventQueue), tsunami(t)
{
    DPRINTF(MC146818, "RTC Event Initilizing\n");
    schedule(curTick + ticksPerSecond/RTC_RATE);
}

void
TsunamiIO::RTCEvent::process()
{
    DPRINTF(MC146818, "Timer Interrupt\n");
    schedule(curTick + ticksPerSecond/RTC_RATE);
    //Actually interrupt the processor here
    if (!tsunami->cchip->RTCInterrupting) {
        tsunami->cchip->misc |= 1 << 7;
        tsunami->cchip->RTCInterrupting = true;
        tsunami->intrctrl->post(0, TheISA::INTLEVEL_IRQ2, 0);
    }
}

const char *
TsunamiIO::RTCEvent::description()
{
    return "tsunami RTC 1024Hz interrupt";
}

// Timer Event for PIT Timers
TsunamiIO::ClockEvent::ClockEvent()
    : Event(&mainEventQueue)
{
    DPRINTF(Tsunami, "Clock Event Initilizing\n");
    mode = 0;
}

void
TsunamiIO::ClockEvent::process()
{
    DPRINTF(Tsunami, "Timer Interrupt\n");
    if (mode == 0)
       status = 0x20; // set bit that linux is looking for
    else
        schedule(curTick + interval);
}

void
TsunamiIO::ClockEvent::Program(int count)
{
    DPRINTF(Tsunami, "Timer set to curTick + %d\n", count);
    // should be count * (cpufreq/pitfreq)
    interval = count * ticksPerSecond/1193180UL;
    schedule(curTick + interval);
    status = 0;
}

const char *
TsunamiIO::ClockEvent::description()
{
    return "tsunami 8254 Interval timer";
}

void
TsunamiIO::ClockEvent::ChangeMode(uint8_t md)
{
    mode = md;
}

uint8_t
TsunamiIO::ClockEvent::Status()
{
    return status;
}




TsunamiIO::TsunamiIO(const string &name, Tsunami *t, time_t init_time,
                       Addr addr, Addr mask, uint32_t f, MemoryController *mmu)
    : MmapDevice(name, addr, mask, mmu), tsunami(t), rtc(t), freq(f)
{
    timerData = 0;
    set_time(init_time == 0 ? time(NULL) : init_time);
    uip = 1;
}

void
TsunamiIO::set_time(time_t t)
{
    gmtime_r(&t, &tm);
    DPRINTFN("Real-time clock set to %s", asctime(&tm));
}

Fault
TsunamiIO::read(MemReqPtr req, uint8_t *data)
{
    DPRINTF(Tsunami, "io read  va=%#x size=%d IOPorrt=%#x\n",
            req->vaddr, req->size, req->vaddr & 0xfff);

    Addr daddr = (req->paddr & addr_mask);
//    ExecContext *xc = req->xc;
//    int cpuid = xc->cpu_id;

    switch(req->size) {
        case sizeof(uint8_t):
            switch(daddr) {
                case TSDEV_TMR_CTL:
                    *(uint8_t*)data = timer2.Status();
                    return No_Fault;
                case TSDEV_RTC_DATA:
                    switch(RTCAddress) {
                        case RTC_CONTROL_REGISTERA:
                            *(uint8_t*)data = uip << 7 | 0x26;
                            uip = !uip;
                            return No_Fault;
                        case RTC_CONTROL_REGISTERB:
                            // DM and 24/12 and UIE
                            *(uint8_t*)data = 0x46;
                            return No_Fault;
                        case RTC_CONTROL_REGISTERC:
                            // If we want to support RTC user access in linux
                            // This won't work, but for now it's fine
                            *(uint8_t*)data = 0x00;
                            return No_Fault;
                        case RTC_CONTROL_REGISTERD:
                            panic("RTC Control Register D not implemented");
                        case RTC_SECOND:
                            *(uint8_t *)data = tm.tm_sec;
                            return No_Fault;
                        case RTC_MINUTE:
                            *(uint8_t *)data = tm.tm_min;
                            return No_Fault;
                        case RTC_HOUR:
                            *(uint8_t *)data = tm.tm_hour;
                            return No_Fault;
                        case RTC_DAY_OF_WEEK:
                            *(uint8_t *)data = tm.tm_wday;
                            return No_Fault;
                        case RTC_DAY_OF_MONTH:
                            *(uint8_t *)data = tm.tm_mday;
                        case RTC_MONTH:
                            *(uint8_t *)data = tm.tm_mon + 1;
                            return No_Fault;
                        case RTC_YEAR:
                            *(uint8_t *)data = tm.tm_year - UNIX_YEAR_OFFSET;
                            return No_Fault;
                        default:
                            panic("Unknown RTC Address\n");
                    }

                default:
                    panic("I/O Read - va%#x size %d\n", req->vaddr, req->size);
            }
        case sizeof(uint16_t):
        case sizeof(uint32_t):
        case sizeof(uint64_t):
        default:
            panic("I/O Read - invalid size - va %#x size %d\n", req->vaddr, req->size);
    }
     panic("I/O Read - va%#x size %d\n", req->vaddr, req->size);

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
                case TSDEV_TMR_CTL:
                    return No_Fault;
                case TSDEV_TMR2_CTL:
                    if ((*(uint8_t*)data & 0x30) != 0x30)
                        panic("Only L/M write supported\n");

                    switch(*(uint8_t*)data >> 6) {
                        case 0:
                            timer0.ChangeMode((*(uint8_t*)data & 0xF) >> 1);
                            break;
                        case 2:
                            timer2.ChangeMode((*(uint8_t*)data & 0xF) >> 1);
                            break;
                        default:
                            panic("Read Back Command not implemented\n");
                    }
                    return No_Fault;
                case TSDEV_TMR2_DATA:
                        /* two writes before we actually start the Timer
                           so I set a flag in the timerData */
                        if(timerData & 0x1000) {
                            timerData &= 0x1000;
                            timerData += *(uint8_t*)data << 8;
                            timer2.Program(timerData);
                        } else {
                            timerData = *(uint8_t*)data;
                            timerData |= 0x1000;
                        }
                        return No_Fault;
                case TSDEV_TMR0_DATA:
                        /* two writes before we actually start the Timer
                           so I set a flag in the timerData */
                        if(timerData & 0x1000) {
                            timerData &= 0x1000;
                            timerData += *(uint8_t*)data << 8;
                            timer0.Program(timerData);
                        } else {
                            timerData = *(uint8_t*)data;
                            timerData |= 0x1000;
                        }
                        return No_Fault;
                case TSDEV_RTC_ADDR:
                        RTCAddress = *(uint8_t*)data;
                        return No_Fault;
                case TSDEV_RTC_DATA:
                        panic("RTC Write not implmented (rtc.o won't work)\n");
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

    SimObjectParam<Tsunami *> tsunami;
    Param<time_t> time;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;
    Param<uint32_t> frequency;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiIO)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

    INIT_PARAM(tsunami, "Tsunami"),
    INIT_PARAM_DFLT(time, "System time to use "
            "(0 for actual time, default is 1/1/06", ULL(1136073600)),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask"),
    INIT_PARAM(frequency, "clock interrupt frequency")

END_INIT_SIM_OBJECT_PARAMS(TsunamiIO)

CREATE_SIM_OBJECT(TsunamiIO)
{
    return new TsunamiIO(getInstanceName(), tsunami, time,  addr,
                         mask, frequency, mmu);
}

REGISTER_SIM_OBJECT("TsunamiIO", TsunamiIO)
