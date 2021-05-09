/*
 * Copyright (c) 2014, 2017 ARM Limited
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

#ifndef __DEV_SERIAL_HH__
#define __DEV_SERIAL_HH__

#include <functional>

#include "sim/sim_object.hh"

namespace gem5
{

struct SerialDeviceParams;
struct SerialNullDeviceParams;

/**
 * Base class for serial devices such as terminals.
 *
 * This class provides a unified interface that all serial (RS232 or
 * similar) devices must implement. A device can be wired to exactly
 * one host serial interface (serial port model).
 *
 * SerialDevices are passive devices that are <i>driven</i> by the
 * serial interface using the writeData(c) (the interface sends a
 * character) and readData() (the interface reads a character)
 * methods. Serial devices need to override these methods to
 * communicate with the host interface layer.
 *
 * To implement basic flow control, serial devices must implement the
 * dataAvailable() method. This method returns true if a valid
 * character can be read using the readData() method. When data
 * becomes available, the serial device must call the
 * notifyInterface() method to send a callback to the interface layer.
 *
 * To send a character (host to device), the interface layer calls
 * writeData(char) to send a character to the serial device.
 *
 * To read a character (device to host), the interface layer calls
 * dataAvailable() to determine if there is a character pending. If
 * there is data available, it immediately calls readData() to get the
 * character. The receive loop in the serial device typically looks
 * like this:
 *
 * \code{.cc}
 * while (device.dataAvailable()) {
 *    printf("%c", (int)device.readData());
 * }
 * \endcode
 *
 * To avoid polling, the interface layer may register a data available
 * callback using the regInterfaceCallback() method. The device uses
 * this callback to notify the interface layer whenever there is new
 * data pending. Note that devices will normally only notify the
 * interface layer when there is a state transition in the
 * device. E.g., the dataAvailable() transitions from false to
 * true. This means that there can be multiple pending characters when
 * the interface layer receives the callback.
 */
class SerialDevice : public SimObject
{
  public:
    SerialDevice(const SerialDeviceParams &p);
    ~SerialDevice();

  public: // Serial device API (UART->Device)
    /**
     * Register a data available callback into the host interface layer.
     *
     * Serial devices need to call the underlying host interface layer
     * to inform it of state change such as pending data that can be
     * read from the device by the interface layer using the readData()
     * method. The interface layer may use this method to register a
     * callback that informs it of pending data.
     *
     * @param c Callback from interface layer.
     */
    void regInterfaceCallback(const std::function<void()> &callback);

    /**
     * Check if there is pending data from the serial device.
     *
     * @return true if there is data pending that can be read using
     * the readData() method.
     */
    virtual bool dataAvailable() const = 0;

    /**
     * Transmit a character from the host interface to the device.
     *
     * @param c Received data.
     */
    virtual void writeData(uint8_t c) = 0;

    /**
     * Read a character from the device.
     *
     * @return Character from the device's output buffer, undefined if
     * no data is pending.
     */
    virtual uint8_t readData() = 0;

  protected:
    /** Notify the host interface of pending data. */
    void notifyInterface();

  private:
    /** Currently regisxtered host interface layer callback */
    std::function<void()> interfaceCallback;
};

/**
 * Dummy serial device that discards all data sent to it.
 */
class SerialNullDevice : public SerialDevice
{
  public:
    SerialNullDevice(const SerialNullDeviceParams &p);

  public:
    bool dataAvailable() const override { return false; }
    void writeData(uint8_t c) override {};
    uint8_t readData() override;
};

} // namespace gem5

#endif // __DEV_SERIAL_HH__
