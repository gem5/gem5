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
 * Declaration of a VNC input
 */

#ifndef __BASE_VNC_VNC_INPUT_HH__
#define __BASE_VNC_VNC_INPUT_HH__

#include <iostream>
#include <memory>

#include "base/compiler.hh"
#include "base/imgwriter.hh"
#include "params/VncInput.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class OutputDirectory;

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

class VncInput : public SimObject
{
  public:
    /** Client -> Server message IDs */
    enum ClientMessages
    {
        ClientSetPixelFormat = 0,
        ClientSetEncodings = 2,
        ClientFrameBufferUpdate = 3,
        ClientKeyEvent = 4,
        ClientPointerEvent = 5,
        ClientCutText = 6
    };

    struct GEM5_PACKED PixelFormat
    {
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
    };

    struct GEM5_PACKED PixelFormatMessage
    {
        uint8_t type;
        uint8_t padding[3];
        PixelFormat px;
    };

    struct GEM5_PACKED PixelEncodingsMessage
    {
        uint8_t type;
        uint8_t padding;
        uint16_t num_encodings;
    };

    struct GEM5_PACKED FrameBufferUpdateReq
    {
        uint8_t type;
        uint8_t incremental;
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
    };

    struct GEM5_PACKED KeyEventMessage
    {
        uint8_t type;
        uint8_t down_flag;
        uint8_t padding[2];
        uint32_t key;
    };

    struct GEM5_PACKED PointerEventMessage
    {
        uint8_t type;
        uint8_t button_mask;
        uint16_t x;
        uint16_t y;
    };

    struct GEM5_PACKED ClientCutTextMessage
    {
        uint8_t type;
        uint8_t padding[3];
        uint32_t length;
    };

    typedef VncInputParams Params;
    VncInput(const Params &p);

    /** Set the address of the frame buffer we are going to show.
     * To avoid copying, just have the display controller
     * tell us where the data is instead of constanly copying it around
     * @param rfb frame buffer that we're going to use
     */
    virtual void setFrameBuffer(const FrameBuffer *rfb);

    /** Set up the device that would like to receive notifications when keys
     * are pressed in the vnc client keyboard
     * @param _keyboard an object that derrives from VncKeyboard
     */
    void
    setKeyboard(VncKeyboard *_keyboard)
    {
        keyboard = _keyboard;
    }

    /** Setup the device that would like to receive notifications when mouse
     * movements or button presses are received from the vnc client.
     * @param _mouse an object that derrives from VncMouse
     */
    void
    setMouse(VncMouse *_mouse)
    {
        mouse = _mouse;
    }

    /** What is the width of the screen we're displaying.
     * This is used for pointer/tablet devices that need to know to calculate
     * the correct value to send to the device driver.
     * @return the width of the simulated screen
     */
    uint16_t
    videoWidth() const
    {
        return _videoWidth;
    }

    /** What is the height of the screen we're displaying.
     * This is used for pointer/tablet devices that need to know to calculate
     * the correct value to send to the device driver.
     * @return the height of the simulated screen
     */
    uint16_t
    videoHeight() const
    {
        return _videoHeight;
    }

    /** The frame buffer uses this call to notify the vnc server that
     * the frame buffer has been updated and a new image needs to be sent to
     * the client
     */
    virtual void setDirty();

  protected:
    virtual void frameBufferResized(){};

    /** The device to notify when we get key events */
    VncKeyboard *keyboard;

    /** The device to notify when we get mouse events */
    VncMouse *mouse;

    /** pointer to the actual data that is stored in the frame buffer device */
    const FrameBuffer *fb;

    /** the width of the frame buffer we are sending to the client */
    uint16_t _videoWidth;

    /** the height of the frame buffer we are sending to the client */
    uint16_t _videoHeight;

    /** Flag indicating whether to capture snapshots of frame buffer or not */
    bool captureEnabled;

    /** Current frame number being captured to a file */
    int captureCurrentFrame;

    /** Directory to store captured frames to */
    OutputDirectory *captureOutputDirectory;

    /** Computed hash of the last captured frame */
    uint64_t captureLastHash;

    /** Cached ImgWriter object for writing out frame buffers to file */
    std::unique_ptr<ImgWriter> captureImage;

    /** image format */
    enums::ImageFormat imgFormat;

    /** Captures the current frame buffer to a file */
    void captureFrameBuffer();
};

} // namespace gem5

#endif
