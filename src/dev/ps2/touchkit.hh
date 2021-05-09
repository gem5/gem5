/*
 * Copyright (c) 2010, 2017-2018 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
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
 */

#ifndef __DEV_PS2_TOUCHKIT_HH__
#define __DEV_PS2_TOUCHKIT_HH__

#include "base/compiler.hh"
#include "base/vnc/vncinput.hh"
#include "dev/ps2/device.hh"

namespace gem5
{

struct PS2TouchKitParams;

namespace ps2
{

class TouchKit : public Device, public VncMouse
{
  protected:
    enum PS2Commands
    {
        TpReadId = 0xE1,
        TouchKitDiag = 0x0A,
    };

    enum TKCommands
    {
        TouchKitActive = 'A',
        TouchKitFWRev = 'D',
        TouchKitCtrlType = 'E',
    };

  public:
    TouchKit(const PS2TouchKitParams &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected: // from Device
    bool recv(const std::vector<uint8_t> &data) override;

  public: // VncMouse
    void mouseAt(uint16_t x, uint16_t y, uint8_t buttons) override;

  protected:
    bool recvTouchKit(const std::vector<uint8_t> &data);
    void sendTouchKit(const uint8_t *data, size_t size);
    void sendTouchKit(uint8_t data) { sendTouchKit(&data, 1); }

    /** The vnc server we're connected to (if any) */
    VncInput *const vnc;

    /** Is the device enabled? */
    bool enabled;

    /**
     * Has the driver enabled TouchKit mode?  The model suppresses
     * touch event generation until this is true.
     */
    bool touchKitEnabled;
};

} // namespace ps2
} // namespace gem5

GEM5_DEPRECATED_CLASS(PS2TouchKit, gem5::ps2::TouchKit);

#endif // __DEV_PS2_TOUCHKIT_HH__
