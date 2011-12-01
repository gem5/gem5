/*
 * Copyright (c) 2010 ARM Limited
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
 *
 * Authors: Ali Saidi
 *          William Wang
 */

/** @file
 * Declaration of a VNC server
 */

#ifndef __DEV_VNC_SERVER_HH__
#define __DEV_VNC_SERVER_HH__

#include <iostream>

#include "base/vnc/convert.hh"
#include "base/bitmap.hh"
#include "base/circlebuf.hh"
#include "base/pollevent.hh"
#include "base/socket.hh"
#include "cpu/intr_control.hh"
#include "params/VncServer.hh"
#include "sim/sim_object.hh"


/**
 * A device that expects to receive input from the vnc server should derrive
 * (through mulitple inheritence if necessary from VncKeyboard or VncMouse
 * and call setKeyboard() or setMouse() respectively on the vnc server.
 */
class VncKeyboard
{
  public:
    /**
     * Called when the vnc server receives a key press event from the
     * client.
     * @param key the key passed is an x11 keysym
     * @param down is the key now down or up?
     */
    virtual void keyPress(uint32_t key, bool down) = 0;
};

class VncMouse
{
  public:
    /**
     * called whenever the mouse moves or it's button state changes
     * buttons is a simple mask with each button (0-8) corresponding to
     * a bit position in the byte with 1 being down and 0 being up
     * @param x the x position of the mouse
     * @param y the y position of the mouse
     * @param buttos the button state as described above
     */
    virtual void mouseAt(uint16_t x, uint16_t y, uint8_t buttons) = 0;
};

class VncServer : public SimObject
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

    /** Client -> Server message IDs */
    enum ClientMessages {
        ClientSetPixelFormat    = 0,
        ClientSetEncodings      = 2,
        ClientFrameBufferUpdate = 3,
        ClientKeyEvent          = 4,
        ClientPointerEvent      = 5,
        ClientCutText           = 6
    };

    /** Server -> Client message IDs */
    enum ServerMessages {
        ServerFrameBufferUpdate     = 0,
        ServerSetColorMapEntries    = 1,
        ServerBell                  = 2,
        ServerCutText               = 3
    };

    /** Encoding types */
    enum EncodingTypes {
        EncodingRaw         = 0,
        EncodingCopyRect    = 1,
        EncodingHextile     = 5,
        EncodingDesktopSize = -223
    };

    /** keyboard/mouse support */
    enum MouseEvents {
        MouseLeftButton     = 0x1,
        MouseRightButton    = 0x2,
        MouseMiddleButton   = 0x4
    };

    const char* vncVersion() const
    {
        return "RFB 003.008\n";
    }

    enum ConnectionState {
        WaitForProtocolVersion,
        WaitForSecurityResponse,
        WaitForClientInit,
        InitializationPhase,
        NormalPhase
    };

    struct PixelFormat {
        uint8_t bpp;
        uint8_t depth;
        uint8_t bigendian;
        uint8_t truecolor;
        uint16_t redmax;
        uint16_t greenmax;
        uint16_t bluemax;
        uint8_t redshift;
        uint8_t greenshift;
        uint8_t blueshift;
        uint8_t padding[3];
    } M5_ATTR_PACKED;

    struct ServerInitMsg {
        uint16_t fbWidth;
        uint16_t fbHeight;
        PixelFormat px;
        uint32_t namelen;
        char name[2]; // just to put M5 in here
    } M5_ATTR_PACKED;

    struct PixelFormatMessage {
        uint8_t type;
        uint8_t padding[3];
        PixelFormat px;
    } M5_ATTR_PACKED;

    struct PixelEncodingsMessage {
        uint8_t type;
        uint8_t padding;
        uint16_t num_encodings;
    } M5_ATTR_PACKED;

    struct FrameBufferUpdateReq {
        uint8_t type;
        uint8_t incremental;
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
    } M5_ATTR_PACKED;

    struct KeyEventMessage {
        uint8_t type;
        uint8_t down_flag;
        uint8_t padding[2];
        uint32_t key;
    } M5_ATTR_PACKED;

    struct PointerEventMessage {
        uint8_t type;
        uint8_t button_mask;
        uint16_t x;
        uint16_t y;
    } M5_ATTR_PACKED;

    struct ClientCutTextMessage {
        uint8_t type;
        uint8_t padding[3];
        uint32_t length;
    } M5_ATTR_PACKED;

    struct FrameBufferUpdate {
        uint8_t type;
        uint8_t padding;
        uint16_t num_rects;
    } M5_ATTR_PACKED;

    struct FrameBufferRect {
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        int32_t encoding;
    } M5_ATTR_PACKED;

    struct ServerCutText {
        uint8_t type;
        uint8_t padding[3];
        uint32_t length;
    } M5_ATTR_PACKED;

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

    ListenSocket listener;

    void listen(int port);
    void accept();
    void data();
    void detach();

  public:
    typedef VncServerParams Params;
    VncServer(const Params *p);
    ~VncServer();

    // RFB
  protected:

    /** The rfb prototol state the connection is in */
    ConnectionState curState;

    /** the width of the frame buffer we are sending to the client */
    uint16_t _videoWidth;

    /** the height of the frame buffer we are sending to the client */
    uint16_t _videoHeight;

    /** pointer to the actual data that is stored in the frame buffer device */
    uint8_t* clientRfb;

    /** The device to notify when we get key events */
    VncKeyboard *keyboard;

    /** The device to notify when we get mouse events */
    VncMouse *mouse;

    /** An update needs to be sent to the client. Without doing this the
     * client will constantly request data that is pointless */
    bool sendUpdate;

    /** The one and only pixel format we support */
    PixelFormat pixelFormat;

    /** If the vnc client supports receiving raw data. It always should */
    bool supportsRawEnc;

    /** If the vnc client supports the desktop resize command */
    bool supportsResizeEnc;

    /** The mode of data we're getting frame buffer in */
    VideoConvert::Mode videoMode;

    /** The video converter that transforms data for us */
    VideoConvert *vc;

    /** Flag indicating whether to capture snapshots of frame buffer or not */
    bool captureEnabled;

    /** Current frame number being captured to a file */
    int captureCurrentFrame;

    /** Directory to store captured frames to */
    std::string captureOutputDirectory;

    /** Computed hash of the last captured frame */
    uint64_t captureLastHash;

    /** Cached bitmap object for writing out frame buffers to file */
    Bitmap *captureBitmap;

  protected:
    /** Captures the current frame buffer to a file */
    void captureFrameBuffer();

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
     * @return length read
     */
    size_t read(uint8_t *buf, size_t len);

    /** Read len -1 bytes from the client into the buffer provided + 1
     * assert that we read enough bytes. This function exists to handle
     * reading all of the protocol structs above when we've already read
     * the first byte which describes which one we're reading
     * @param buf the address of the buffer to add one to and read data into
     * @param len the amount of data  + 1 to read
     * @return length read
     */
    size_t read1(uint8_t *buf, size_t len);


    /** Templated version of the read function above to
     * read simple data to the client
     * @param val data to recv from the client
     */
    template <typename T> size_t read(T* val);


    /** Write a buffer to the client.
     * @param buf buffer to send
     * @param len length of the buffer
     * @return number of bytes sent
     */
    size_t write(const uint8_t *buf, size_t len);

    /** Templated version of the write function above to
     * write simple data to the client
     * @param val data to send to the client
     */
    template <typename T> size_t write(T* val);

    /** Send a string to the client
     * @param str string to transmit
     */
    size_t write(const char* str);

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

  public:
    /** Set the address of the frame buffer we are going to show.
     * To avoid copying, just have the display controller
     * tell us where the data is instead of constanly copying it around
     * @param rfb frame buffer that we're going to use
     */
    void
    setFramebufferAddr(uint8_t* rfb)
    {
        clientRfb = rfb;
    }

    /** Set up the device that would like to receive notifications when keys are
     * pressed in the vnc client keyboard
     * @param _keyboard an object that derrives from VncKeyboard
     */
    void setKeyboard(VncKeyboard *_keyboard) { keyboard = _keyboard; }

    /** Setup the device that would like to receive notifications when mouse
     * movements or button presses are received from the vnc client.
     * @param _mouse an object that derrives from VncMouse
     */
    void setMouse(VncMouse *_mouse) { mouse = _mouse; }

    /** The frame buffer uses this call to notify the vnc server that
     * the frame buffer has been updated and a new image needs to be sent to the
     * client
     */
    void
    setDirty()
    {
        sendUpdate = true;
        if (captureEnabled)
            captureFrameBuffer();
        sendFrameBufferUpdate();
    }

    /** What is the width of the screen we're displaying.
     * This is used for pointer/tablet devices that need to know to calculate
     * the correct value to send to the device driver.
     * @return the width of the simulated screen
     */
    uint16_t videoWidth() { return _videoWidth; }

    /** What is the height of the screen we're displaying.
     * This is used for pointer/tablet devices that need to know to calculate
     * the correct value to send to the device driver.
     * @return the height of the simulated screen
     */
    uint16_t videoHeight() { return _videoHeight; }

    /** Set the mode of the data the frame buffer will be sending us
     * @param mode the mode
     */
    void setFrameBufferParams(VideoConvert::Mode mode, int width, int height);
};

#endif
