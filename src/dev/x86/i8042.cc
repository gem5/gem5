/*
 * Copyright (c) 2008 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include "dev/x86/i8042.hh"

#include "base/bitunion.hh"
#include "debug/I8042.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"

/**
 * Note: For details on the implementation see
 * https://wiki.osdev.org/%228042%22_PS/2_Controller
 */

// The 8042 has a whopping 32 bytes of internal RAM.
const uint8_t RamSize = 32;
const uint8_t NumOutputBits = 14;


X86ISA::I8042::I8042(Params *p)
    : BasicPioDevice(p, 0), // pioSize arg is dummy value... not used
      latency(p->pio_latency),
      dataPort(p->data_port), commandPort(p->command_port),
      statusReg(0), commandByte(0), dataReg(0), lastCommand(NoCommand),
      mouseIntPin(p->mouse_int_pin), keyboardIntPin(p->keyboard_int_pin),
      mouse(p->mouse), keyboard(p->keyboard)
{
    fatal_if(!mouse, "The i8042 model requires a mouse instance");
    fatal_if(!keyboard, "The i8042 model requires a keyboard instance");

    statusReg.passedSelfTest = 1;
    statusReg.commandLast = 1;
    statusReg.keyboardUnlocked = 1;

    commandByte.convertScanCodes = 1;
    commandByte.passedSelfTest = 1;
    commandByte.keyboardFullInt = 1;
}


AddrRangeList
X86ISA::I8042::getAddrRanges() const
{
    AddrRangeList ranges;
    // TODO: Are these really supposed to be a single byte and not 4?
    ranges.push_back(RangeSize(dataPort, 1));
    ranges.push_back(RangeSize(commandPort, 1));
    return ranges;
}

void
X86ISA::I8042::writeData(uint8_t newData, bool mouse)
{
    DPRINTF(I8042, "Set data %#02x.\n", newData);
    dataReg = newData;
    statusReg.outputFull = 1;
    statusReg.mouseOutputFull = (mouse ? 1 : 0);
    if (!mouse && commandByte.keyboardFullInt) {
        DPRINTF(I8042, "Sending keyboard interrupt.\n");
        keyboardIntPin->raise();
        //This is a hack
        keyboardIntPin->lower();
    } else if (mouse && commandByte.mouseFullInt) {
        DPRINTF(I8042, "Sending mouse interrupt.\n");
        mouseIntPin->raise();
        //This is a hack
        mouseIntPin->lower();
    }
}

uint8_t
X86ISA::I8042::readDataOut()
{
    uint8_t data = dataReg;
    statusReg.outputFull = 0;
    statusReg.mouseOutputFull = 0;
    if (keyboard->hostDataAvailable()) {
        writeData(keyboard->hostRead(), false);
    } else if (mouse->hostDataAvailable()) {
        writeData(mouse->hostRead(), true);
    }
    return data;
}

Tick
X86ISA::I8042::read(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr addr = pkt->getAddr();
    if (addr == dataPort) {
        uint8_t data = readDataOut();
        //DPRINTF(I8042, "Read from data port got %#02x.\n", data);
        pkt->setLE<uint8_t>(data);
    } else if (addr == commandPort) {
        //DPRINTF(I8042, "Read status as %#02x.\n", (uint8_t)statusReg);
        pkt->setLE<uint8_t>((uint8_t)statusReg);
    } else {
        panic("Read from unrecognized port %#x.\n", addr);
    }
    pkt->makeAtomicResponse();
    return latency;
}

Tick
X86ISA::I8042::write(PacketPtr pkt)
{
    assert(pkt->getSize() == 1);
    Addr addr = pkt->getAddr();
    uint8_t data = pkt->getLE<uint8_t>();
    if (addr == dataPort) {
        statusReg.commandLast = 0;
        switch (lastCommand) {
          case NoCommand:
            keyboard->hostWrite(data);
            if (keyboard->hostDataAvailable())
                writeData(keyboard->hostRead(), false);
            break;
          case WriteToMouse:
            mouse->hostWrite(data);
            if (mouse->hostDataAvailable())
                writeData(mouse->hostRead(), true);
            break;
          case WriteCommandByte:
            commandByte = data;
            DPRINTF(I8042, "Got data %#02x for \"Write "
                    "command byte\" command.\n", data);
            statusReg.passedSelfTest = (uint8_t)commandByte.passedSelfTest;
            break;
          case WriteMouseOutputBuff:
            DPRINTF(I8042, "Got data %#02x for \"Write "
                    "mouse output buffer\" command.\n", data);
            writeData(data, true);
            break;
          case WriteKeyboardOutputBuff:
            DPRINTF(I8042, "Got data %#02x for \"Write "
                    "keyboad output buffer\" command.\n", data);
            writeData(data, false);
            break;
          case WriteOutputPort:
            DPRINTF(I8042, "Got data %#02x for \"Write "
                    "output port\" command.\n", data);
            panic_if(bits(data, 0) != 1, "Reset bit should be 1");
            // Safe to ignore otherwise
            break;
          default:
            panic("Data written for unrecognized "
                    "command %#02x\n", lastCommand);
        }
        lastCommand = NoCommand;
    } else if (addr == commandPort) {
        DPRINTF(I8042, "Got command %#02x.\n", data);
        statusReg.commandLast = 1;
        // These purposefully leave off the first byte of the controller RAM
        // so it can be handled specially.
        if (data > ReadControllerRamBase &&
                data < ReadControllerRamBase + RamSize) {
            panic("Attempted to use i8042 read controller RAM command to "
                    "get byte %d.\n", data - ReadControllerRamBase);
        } else if (data > WriteControllerRamBase &&
                data < WriteControllerRamBase + RamSize) {
            panic("Attempted to use i8042 read controller RAM command to "
                    "get byte %d.\n", data - ReadControllerRamBase);
        } else if (data >= PulseOutputBitBase &&
                data < PulseOutputBitBase + NumOutputBits) {
            panic("Attempted to use i8042 pulse output bit command to "
                    "to pulse bit %d.\n", data - PulseOutputBitBase);
        }
        switch (data) {
          case GetCommandByte:
            DPRINTF(I8042, "Getting command byte.\n");
            writeData(commandByte);
            break;
          case WriteCommandByte:
            DPRINTF(I8042, "Setting command byte.\n");
            lastCommand = WriteCommandByte;
            break;
          case CheckForPassword:
            panic("i8042 \"Check for password\" command not implemented.\n");
          case LoadPassword:
            panic("i8042 \"Load password\" command not implemented.\n");
          case CheckPassword:
            panic("i8042 \"Check password\" command not implemented.\n");
          case DisableMouse:
            DPRINTF(I8042, "Disabling mouse at controller.\n");
            commandByte.disableMouse = 1;
            break;
          case EnableMouse:
            DPRINTF(I8042, "Enabling mouse at controller.\n");
            commandByte.disableMouse = 0;
            break;
          case TestMouse:
            panic("i8042 \"Test mouse\" command not implemented.\n");
          case SelfTest:
            panic("i8042 \"Self test\" command not implemented.\n");
          case InterfaceTest:
            panic("i8042 \"Interface test\" command not implemented.\n");
          case DiagnosticDump:
            panic("i8042 \"Diagnostic dump\" command not implemented.\n");
          case DisableKeyboard:
            DPRINTF(I8042, "Disabling keyboard at controller.\n");
            commandByte.disableKeyboard = 1;
            break;
          case EnableKeyboard:
            DPRINTF(I8042, "Enabling keyboard at controller.\n");
            commandByte.disableKeyboard = 0;
            break;
          case ReadInputPort:
            panic("i8042 \"Read input port\" command not implemented.\n");
          case ContinuousPollLow:
            panic("i8042 \"Continuous poll low\" command not implemented.\n");
          case ContinuousPollHigh:
            panic("i8042 \"Continuous poll high\" command not implemented.\n");
          case ReadOutputPort:
            panic("i8042 \"Read output port\" command not implemented.\n");
          case WriteOutputPort:
            lastCommand = WriteOutputPort;
            break;
          case WriteKeyboardOutputBuff:
            lastCommand = WriteKeyboardOutputBuff;
            break;
          case WriteMouseOutputBuff:
            DPRINTF(I8042, "Got command to write to mouse output buffer.\n");
            lastCommand = WriteMouseOutputBuff;
            break;
          case WriteToMouse:
            DPRINTF(I8042, "Expecting mouse command.\n");
            lastCommand = WriteToMouse;
            break;
          case DisableA20:
            panic("i8042 \"Disable A20\" command not implemented.\n");
          case EnableA20:
            panic("i8042 \"Enable A20\" command not implemented.\n");
          case ReadTestInputs:
            panic("i8042 \"Read test inputs\" command not implemented.\n");
          case SystemReset:
            panic("i8042 \"System reset\" command not implemented.\n");
          default:
            warn("Write to unknown i8042 "
                    "(keyboard controller) command port.\n");
        }
    } else {
        panic("Write to unrecognized port %#x.\n", addr);
    }
    pkt->makeAtomicResponse();
    return latency;
}

void
X86ISA::I8042::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(dataPort);
    SERIALIZE_SCALAR(commandPort);
    SERIALIZE_SCALAR(statusReg);
    SERIALIZE_SCALAR(commandByte);
    SERIALIZE_SCALAR(dataReg);
    SERIALIZE_SCALAR(lastCommand);
}

void
X86ISA::I8042::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(dataPort);
    UNSERIALIZE_SCALAR(commandPort);
    UNSERIALIZE_SCALAR(statusReg);
    UNSERIALIZE_SCALAR(commandByte);
    UNSERIALIZE_SCALAR(dataReg);
    UNSERIALIZE_SCALAR(lastCommand);
}

X86ISA::I8042 *
I8042Params::create()
{
    return new X86ISA::I8042(this);
}
