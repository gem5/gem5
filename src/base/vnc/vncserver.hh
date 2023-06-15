/*
 * Copyright (c) 2010, 2015 ARM Limited
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

/** @file
 * Declaration of a VNC server
 */

#ifndef __BASE_VNC_VNC_SERVER_HH__
#define __BASE_VNC_VNC_SERVER_HH__

#include <iostream>

#include "base/circlebuf.hh"
#include "base/compiler.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "base/vnc/vncinput.hh"
#include "params/VncServer.hh"
#include "sim/sim_object.hh"

/** @file
 * Declaration of a VNC server
 */

namespace gem5
{

class VncServer : public VncInput
{
  public:

    /**
     * \defgroup VncConstants A set of constants and structs from the VNC spec
     * @{
     */
    /** Authentication modes */
    const static uint32_t AuthInvalid = 0;
    const static uint32_t AuthNone    = 1;

    /** Error conditions */
    const static uint32_t VncOK   = 0;

    /** Server -> Client message IDs */
    enum ServerMessages
    {
        ServerFrameBufferUpdate     = 0,
        ServerSetColorMapEntries    = 1,
        ServerBell                  = 2,
        ServerCutText               = 3
    };

    /** Encoding types */
    enum EncodingTypes
    {
        EncodingRaw         = 0,
        EncodingCopyRect    = 1,
        EncodingHextile     = 5,
        EncodingDesktopSize = -223
    };

    /** keyboard/mouse support */
    enum MouseEvents
    {
        MouseLeftButton     = 0x1,
        MouseRightButton    = 0x2,
        MouseMiddleButton   = 0x4
    };

    const char* vncVersion() const
    {
        return "RFB 003.008\n";
    }

    enum ConnectionState
    {
        WaitForProtocolVersion,
        WaitForSecurityResponse,
        WaitForClientInit,
        InitializationPhase,
        NormalPhase
    };

    struct GEM5_PACKED ServerInitMsg
    {
        uint16_t fbWidth;
        uint16_t fbHeight;
        PixelFormat px;
        uint32_t namelen;
        char name[2]; // just to put M5 in here
    };

    struct GEM5_PACKED FrameBufferUpdate
    {
        uint8_t type;
        uint8_t padding;
        uint16_t num_rects;
    };

    struct GEM5_PACKED FrameBufferRect
    {
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        int32_t encoding;
    };

    struct GEM5_PACKED ServerCutText
    {
        uint8_t type;
        uint8_t padding[3];
        uint32_t length;
    };

    /** @} */

  protected:
    /** ListenEvent to accept a vnc client connection */
    class ListenEvent: public PollEvent
    {
      protected:
        VncServer *vncserver;

      public:
        ListenEvent(VncServer *vs, int fd, int e);
        void process(int revent);
    };

    friend class ListenEvent;
    ListenEvent *listenEvent;

    /** DataEvent to read data from vnc */
    class DataEvent: public PollEvent
    {
      protected:
        VncServer *vncserver;

      public:
        DataEvent(VncServer *vs, int fd, int e);
        void process(int revent);
    };

    friend class DataEvent;
    DataEvent *dataEvent;

    int number;
    int dataFd; // data stream file describer

    ListenSocketPtr listener;

    void listen();
    void accept();
    void data();
    void detach();

  public:
    typedef VncServerParams Params;
    VncServer(const Params &p);
    ~VncServer();

    // RFB
  protected:

    /** The rfb prototol state the connection is in */
    ConnectionState curState;

    /** An update needs to be sent to the client. Without doing this the
     * client will constantly request data that is pointless */
    bool sendUpdate;

    /** The one and only pixel format we support */
    PixelFormat pixelFormat;

    /** If the vnc client supports receiving raw data. It always should */
    bool supportsRawEnc;

    /** If the vnc client supports the desktop resize command */
    bool supportsResizeEnc;

  protected:
    /**
     * vnc client Interface
     */

    /** Send an error message to the client
     * @param error_msg text to send describing the error
     */
    void sendError(const char* error_msg);

    /** Read some data from the client
     * @param buf the data to read
     * @param len the amount of data to read
     * @return whether the read was successful
     */
    bool read(uint8_t *buf, size_t len);

    /** Read len -1 bytes from the client into the buffer provided + 1
     * assert that we read enough bytes. This function exists to handle
     * reading all of the protocol structs above when we've already read
     * the first byte which describes which one we're reading
     * @param buf the address of the buffer to add one to and read data into
     * @param len the amount of data  + 1 to read
     * @return whether the read was successful.
     */
    bool read1(uint8_t *buf, size_t len);


    /** Templated version of the read function above to
     * read simple data to the client
     * @param val data to recv from the client
     */
    template <typename T> bool read(T* val);


    /** Write a buffer to the client.
     * @param buf buffer to send
     * @param len length of the buffer
     * @return whether the write was successful
     */
    bool write(const uint8_t *buf, size_t len);

    /** Templated version of the write function above to
     * write simple data to the client
     * @param val data to send to the client
     */
    template <typename T> bool write(T* val);

    /** Send a string to the client
     * @param str string to transmit
     */
    bool write(const char* str);

    /** Check the client's protocol verion for compatibility and send
     * the security types we support
     */
    void checkProtocolVersion();

    /** Check that the security exchange was successful
     */
    void checkSecurity();

    /** Send client our idea about what the frame buffer looks like */
    void sendServerInit();

    /** Send an error message to the client when something goes wrong
     * @param error_msg error to send
     */
    void sendError(std::string error_msg);

    /** Send a updated frame buffer to the client.
     * @todo this doesn't do anything smart and just sends the entire image
     */
    void sendFrameBufferUpdate();

    /** Receive pixel foramt message from client and process it. */
    void setPixelFormat();

    /** Receive encodings message from client and process it. */
    void setEncodings();

    /** Receive message from client asking for updated frame buffer */
    void requestFbUpdate();

    /** Receive message from client providing new keyboard input */
    void recvKeyboardInput();

    /** Recv message from client providing new mouse movement or button click */
    void recvPointerInput();

    /**  Receive message from client that there is text in it's paste buffer.
     * This is a no-op at the moment, but perhaps we would want to be able to
     * paste it at some point.
     */
    void recvCutText();

    /** Tell the client that the frame buffer resized. This happens when the
     * simulated system changes video modes (E.g. X11 starts).
     */
    void sendFrameBufferResized();

    static const PixelConverter pixelConverter;

  public:
    void setDirty() override;
    void frameBufferResized() override;
};

} // namespace gem5

#endif
