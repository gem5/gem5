/* $Id$ */

/* @file
 * Tsunami UART
 */

/*
 * Copyright (C) 1998 by the Board of Trustees
 *    of Leland Stanford Junior University.
 * Copyright (C) 1998 Digital Equipment Corporation
 *
 * This file is part of the SimOS distribution.
 * See LICENSE file for terms of the license.
 *
 */

#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/str.hh"	// for to_number
#include "base/trace.hh"
#include "dev/console.hh"
#include "dev/tsunami_uart.hh"
#include "mem/functional_mem/memory_control.hh"
#include "sim/builder.hh"
#include "targetarch/ev5.hh"

using namespace std;

#define CONS_INT_TX   0x01  // interrupt enable / state bits
#define CONS_INT_RX   0x02

TsunamiUart::TsunamiUart(const string &name, SimConsole *c,
                       Addr addr, Addr mask, MemoryController *mmu)
    : MmapDevice(name, addr, mask, mmu),
      cons(c), status_store(0), valid_char(false)
{
}

Fault
TsunamiUart::read(MemReqPtr &req, uint8_t *data)
{
    Addr daddr = req->paddr & addr_mask;
    DPRINTF(TsunamiUart, " read register %#x\n", daddr);

    switch (req->size) {
      case sizeof(uint64_t):
        *(uint64_t *)data = 0;
        break;
      case sizeof(uint32_t):
        *(uint32_t *)data = 0;
        break;
      case sizeof(uint16_t):
        *(uint16_t *)data = 0;
        break;
      case sizeof(uint8_t):
        *(uint8_t *)data = 0;
        break;
    }

    switch (daddr) {
      case 0xD: // Status Register
        {
            int status = cons->intStatus();
            if (!valid_char) {
                valid_char = cons->in(next_char);
                if (!valid_char)
                    status &= ~CONS_INT_RX;
            } else {
                status |= CONS_INT_RX;
            }

            if (status_store == 3) {
                // RR3 stuff? Don't really understand it, btw
                status_store = 0;
                if (status & CONS_INT_TX) {
                    *data = (1 << 4);
                    return No_Fault;
                } else if (status & CONS_INT_RX) {
                    *data = (1 << 5);
                    return No_Fault;
                } else {
                    DPRINTF(TsunamiUart, "spurious read\n");
                    return No_Fault;
                }
            } else {
                int reg = (1 << 2) | (1 << 5) | (1 << 6);
                if (status & CONS_INT_RX)
                    reg |= (1 << 0);
                *data = reg;
                return No_Fault;
            }
            break;
        }

      case 0x8: // Data register (RX)
        if (!valid_char)
            panic("Invalid character");

        DPRINTF(TsunamiUart, "read data register \'%c\' %#02x\n",
                isprint(next_char) ? next_char : ' ', next_char);

        *data = next_char;
        valid_char = false;
        return No_Fault;

      case 0x9: // Interrupt Enable Register
        *data = 0;
        return No_Fault;
    }
    *data = 0;
   // panic("%s: read daddr=%#x type=read *data=%#x\n", name(), daddr, *data);

    return No_Fault;
}

Fault
TsunamiUart::write(MemReqPtr &req, const uint8_t *data)
{
    Addr daddr = req->paddr & addr_mask;

    DPRINTF(TsunamiUart, " write register %#x value %#x\n", daddr, *(uint8_t*)data);
    switch (daddr) {
      case 0xb:
        status_store = *data;
        switch (*data) {
          case 0x03: // going to read RR3
            return No_Fault;

          case 0x28: // Ack of TX
            {
                if ((cons->intStatus() & CONS_INT_TX) == 0)
                    panic("Ack of transmit, though there was no interrupt");

                cons->clearInt(CONS_INT_TX);
                return No_Fault;
            }

          case 0x00:
          case 0x01:
          case 0x12:
            // going to write data???
            return No_Fault;

          default:
            DPRINTF(TsunamiUart, "writing status register %#x \n",
                    *(uint64_t *)data);
            return No_Fault;
        }

      case 0x8: // Data register (TX)
        cons->out(*(uint64_t *)data);
        return No_Fault;
      case 0x9: // DLM
        DPRINTF(TsunamiUart, "writing to DLM/IER %#x\n", *(uint8_t*)data);
        return No_Fault;
      case 0xc: // MCR
        DPRINTF(TsunamiUart, "writing to MCR %#x\n", *(uint8_t*)data);
        return No_Fault;

    }

    return No_Fault;
}

void
TsunamiUart::serialize(ostream &os)
{
    SERIALIZE_SCALAR(status_store);
    SERIALIZE_SCALAR(next_char);
    SERIALIZE_SCALAR(valid_char);
}

void
TsunamiUart::unserialize(Checkpoint *cp, const std::string &section)
{
    UNSERIALIZE_SCALAR(status_store);
    UNSERIALIZE_SCALAR(next_char);
    UNSERIALIZE_SCALAR(valid_char);
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(TsunamiUart)

    SimObjectParam<SimConsole *> console;
    SimObjectParam<MemoryController *> mmu;
    Param<Addr> addr;
    Param<Addr> mask;

END_DECLARE_SIM_OBJECT_PARAMS(TsunamiUart)

BEGIN_INIT_SIM_OBJECT_PARAMS(TsunamiUart)

    INIT_PARAM(console, "The console"),
    INIT_PARAM(mmu, "Memory Controller"),
    INIT_PARAM(addr, "Device Address"),
    INIT_PARAM(mask, "Address Mask")

END_INIT_SIM_OBJECT_PARAMS(TsunamiUart)

CREATE_SIM_OBJECT(TsunamiUart)
{
    return new TsunamiUart(getInstanceName(), console, addr, mask, mmu);
}

REGISTER_SIM_OBJECT("TsunamiUart", TsunamiUart)
