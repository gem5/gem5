/*
 * Copyright (c) 2017-2018 ARM Limited
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
 *          Andreas Sandberg
 */

#ifndef __DEV_PS2_DEVICE_HH__
#define __DEV_PS2_DEVICE_HH__

#include <deque>
#include <vector>

#include "sim/sim_object.hh"

struct PS2DeviceParams;

class PS2Device : public SimObject
{
  public:
    PS2Device(const PS2DeviceParams *p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: /* Host interface */
    /**
     * Register a data available callback into the PS/2 interface
     *
     * @param c Callback instance from host interface
     */
    void hostRegDataAvailable(const std::function<void()> &c);

    /**
     * Check if there is pending data from the PS/2 device.
     *
     * @return true if there is data pending that can be read using
     * the readData() method.
     */
    bool hostDataAvailable() const  { return !outBuffer.empty(); }

    /**
     * Read a character from the device.
     *
     * @return Character from the device's output buffer, undefined if
     * no data is pending.
     */
    uint8_t hostRead();

    /**
     * Transmit a character from the host interface to the device.
     *
     * @param c Received data.
     */
    void hostWrite(uint8_t c);

  protected: /* Device interface */
    /**
     * Data received from host.
     *
     * Data sent to the device is buffered one byte at a time. Each
     * time a byte is added, this function is called and passed the
     * current buffer. It should return true if it has consumed the
     * data and the buffer can be cleared, or false if more data is
     * needed to process the current command.
     *
     * @param data Pending input data (at least one byte)
     * @return false if more data is needed to process the current
     * command, true otherwise.
     */
    virtual bool recv(const std::vector<uint8_t> &data) = 0;

    /**
     * Send data from a PS/2 device to a host
     *
     * @param data Pointer to data array
     * @param size Size of the data array
     */
    void send(const uint8_t *data, size_t size);
    void send(const std::vector<uint8_t> &data) {
        send(data.data(), data.size());
    }

    /**
     * Send a byte of data from a PS/2 device to a host
     *
     * @param data Byte to send
     */
    void send(uint8_t data) { send(&data, 1); }

    /** Send an ACK byte to the host */
    void sendAck();

    /**
     * Output buffer size
     *
     * Device models may use this method to query the size of the
     * output buffer to do rate limiting.
     */
    size_t sendPending() const { return outBuffer.size(); }

  private:
    /** Device -> host FIFO */
    std::deque<uint8_t> outBuffer;

    /** Host -> device buffer */
    std::vector<uint8_t> inBuffer;

    std::function<void()> dataAvailableCallback;
};

#endif // __DEV_PS2_HOUSE_HH__
