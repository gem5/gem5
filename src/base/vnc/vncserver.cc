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
 *
 * Authors: Ali Saidi
 *          William Wang
 */

/** @file
 * Implementiation of a VNC server
 */

#include <sys/ioctl.h>
#include <sys/stat.h>

#if defined(__FreeBSD__)
#include <termios.h>

#else
#include <sys/termios.h>

#endif
#include "base/vnc/vncserver.hh"

#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstddef>

#include "base/atomicio.hh"
#include "base/bitmap.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/socket.hh"
#include "base/trace.hh"
#include "debug/VNC.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"

using namespace std;

const PixelConverter VncServer::pixelConverter(
    4,        // 4 bytes / pixel
    16, 8, 0, // R in [23, 16], G in [15, 8], B in [7, 0]
    8, 8, 8,  // 8 bits / channel
    LittleEndianByteOrder);

/** @file
 * Implementiation of a VNC server
 */

/**
 * Poll event for the listen socket
 */
VncServer::ListenEvent::ListenEvent(VncServer *vs, int fd, int e)
    : PollEvent(fd, e), vncserver(vs)
{
}

void
VncServer::ListenEvent::process(int revent)
{
    vncserver->accept();
}

/**
 * Poll event for the data socket
 */
VncServer::DataEvent::DataEvent(VncServer *vs, int fd, int e)
    : PollEvent(fd, e), vncserver(vs)
{
}

void
VncServer::DataEvent::process(int revent)
{
    if (revent & POLLIN)
        vncserver->data();
    else if (revent & POLLNVAL)
        vncserver->detach();
}

/**
 * VncServer
 */
VncServer::VncServer(const Params *p)
    : VncInput(p), listenEvent(NULL), dataEvent(NULL), number(p->number),
      dataFd(-1), sendUpdate(false),
      supportsRawEnc(false), supportsResizeEnc(false)
{
    if (p->port)
        listen(p->port);

    curState = WaitForProtocolVersion;

    // We currently only support one pixel format. Extract the pixel
    // representation from our PixelConverter instance and keep it
    // around for telling the client and making sure it cooperates
    pixelFormat.bpp = 8 * pixelConverter.length;
    pixelFormat.depth = pixelConverter.depth;
    pixelFormat.bigendian = pixelConverter.byte_order == BigEndianByteOrder;
    pixelFormat.truecolor = 1;
    pixelFormat.redmax = pixelConverter.ch_r.mask;
    pixelFormat.greenmax = pixelConverter.ch_g.mask;
    pixelFormat.bluemax = pixelConverter.ch_b.mask;
    pixelFormat.redshift = pixelConverter.ch_r.offset;
    pixelFormat.greenshift = pixelConverter.ch_g.offset;
    pixelFormat.blueshift = pixelConverter.ch_b.offset;

    DPRINTF(VNC, "Vnc server created at port %d\n", p->port);
}

VncServer::~VncServer()
{
    if (dataFd != -1)
        ::close(dataFd);

    if (listenEvent)
        delete listenEvent;

    if (dataEvent)
        delete dataEvent;
}


//socket creation and vnc client attach
void
VncServer::listen(int port)
{
    if (ListenSocket::allDisabled()) {
        warn_once("Sockets disabled, not accepting vnc client connections");
        return;
    }

    while (!listener.listen(port, true)) {
        DPRINTF(VNC,
                "can't bind address vnc server port %d in use PID %d\n",
                port, getpid());
        port++;
    }

    int p1, p2;
    p2 = name().rfind('.') - 1;
    p1 = name().rfind('.', p2);
    ccprintf(cerr, "Listening for %s connection on port %d\n",
             name().substr(p1 + 1, p2 - p1), port);

    listenEvent = new ListenEvent(this, listener.getfd(), POLLIN);
    pollQueue.schedule(listenEvent);
}

// attach a vnc client
void
VncServer::accept()
{
    // As a consequence of being called from the PollQueue, we might
    // have been called from a different thread. Migrate to "our"
    // thread.
    EventQueue::ScopedMigration migrate(eventQueue());

    if (!listener.islistening())
        panic("%s: cannot accept a connection if not listening!", name());

    int fd = listener.accept(true);
    fatal_if(fd < 0, "%s: failed to accept VNC connection!", name());

    if (dataFd != -1) {
        char message[] = "vnc server already attached!\n";
        atomic_write(fd, message, sizeof(message));
        ::close(fd);
        return;
    }

    dataFd = fd;

    // Send our version number to the client
    write((uint8_t*)vncVersion(), strlen(vncVersion()));

    // read the client response
    dataEvent = new DataEvent(this, dataFd, POLLIN);
    pollQueue.schedule(dataEvent);

    inform("VNC client attached\n");
}

// data called by data event
void
VncServer::data()
{
    // We have new data, see if we can handle it
    size_t len;
    DPRINTF(VNC, "Vnc client message recieved\n");

    switch (curState) {
      case WaitForProtocolVersion:
        checkProtocolVersion();
        break;
      case WaitForSecurityResponse:
        checkSecurity();
        break;
      case WaitForClientInit:
        // Don't care about shared, just need to read it out of the socket
        uint8_t shared;
        len = read(&shared);
        assert(len == 1);

        // Send our idea of the frame buffer
        sendServerInit();

        break;
      case NormalPhase:
        uint8_t message_type;
        len = read(&message_type);
        if (!len) {
            detach();
            return;
        }
        assert(len == 1);

        switch (message_type) {
          case ClientSetPixelFormat:
            setPixelFormat();
            break;
          case ClientSetEncodings:
            setEncodings();
            break;
          case ClientFrameBufferUpdate:
            requestFbUpdate();
            break;
          case ClientKeyEvent:
            recvKeyboardInput();
            break;
          case ClientPointerEvent:
            recvPointerInput();
            break;
          case ClientCutText:
            recvCutText();
            break;
          default:
            panic("Unimplemented message type recv from client: %d\n",
                  message_type);
            break;
        }
        break;
      default:
        panic("Unknown vnc server state\n");
    }
}


// read from socket
size_t
VncServer::read(uint8_t *buf, size_t len)
{
    if (dataFd < 0)
        panic("vnc not properly attached.\n");

    size_t ret;
    do {
        ret = ::read(dataFd, buf, len);
    } while (ret == -1 && errno == EINTR);


    if (ret <= 0){
        DPRINTF(VNC, "Read failed.\n");
        detach();
        return 0;
    }

    return ret;
}

size_t
VncServer::read1(uint8_t *buf, size_t len)
{
    size_t read_len M5_VAR_USED;
    read_len = read(buf + 1, len - 1);
    assert(read_len == len - 1);
    return read_len;
}


template<typename T>
size_t
VncServer::read(T* val)
{
    return read((uint8_t*)val, sizeof(T));
}

// write to socket
size_t
VncServer::write(const uint8_t *buf, size_t len)
{
    if (dataFd < 0)
        panic("Vnc client not properly attached.\n");

    ssize_t ret;
    ret = atomic_write(dataFd, buf, len);

    if (ret < len)
        detach();

    return ret;
}

template<typename T>
size_t
VncServer::write(T* val)
{
    return write((uint8_t*)val, sizeof(T));
}

size_t
VncServer::write(const char* str)
{
    return write((uint8_t*)str, strlen(str));
}

// detach a vnc client
void
VncServer::detach()
{
    if (dataFd != -1) {
        ::close(dataFd);
        dataFd = -1;
    }

    if (!dataEvent || !dataEvent->queued())
        return;

    pollQueue.remove(dataEvent);
    delete dataEvent;
    dataEvent = NULL;
    curState = WaitForProtocolVersion;

    inform("VNC client detached\n");
    DPRINTF(VNC, "detach vnc client %d\n", number);
}

void
VncServer::sendError(const char* error_msg)
{
   uint32_t len = strlen(error_msg);
   write(&len);
   write(error_msg);
}

void
VncServer::checkProtocolVersion()
{
    assert(curState == WaitForProtocolVersion);

    size_t len M5_VAR_USED;
    char version_string[13];

    // Null terminate the message so it's easier to work with
    version_string[12] = 0;

    len = read((uint8_t*)version_string, 12);
    assert(len == 12);

    uint32_t major, minor;

    // Figure out the major/minor numbers
    if (sscanf(version_string, "RFB %03d.%03d\n", &major, &minor) != 2) {
        warn(" Malformed protocol version %s\n", version_string);
        sendError("Malformed protocol version\n");
        detach();
    }

    DPRINTF(VNC, "Client request protocol version %d.%d\n", major, minor);

    // If it's not 3.X we don't support it
    if (major != 3 || minor < 2) {
        warn("Unsupported VNC client version... disconnecting\n");
        uint8_t err = AuthInvalid;
        write(&err);
        detach();
    }
    // Auth is different based on version number
    if (minor < 7) {
        uint32_t sec_type = htobe((uint32_t)AuthNone);
        write(&sec_type);
    } else {
        uint8_t sec_cnt = 1;
        uint8_t sec_type = htobe((uint8_t)AuthNone);
        write(&sec_cnt);
        write(&sec_type);
    }

    // Wait for client to respond
    curState = WaitForSecurityResponse;
}

void
VncServer::checkSecurity()
{
    assert(curState == WaitForSecurityResponse);

    uint8_t security_type;
    size_t len M5_VAR_USED = read(&security_type);

    assert(len == 1);

    if (security_type != AuthNone) {
        warn("Unknown VNC security type\n");
        sendError("Unknown security type\n");
    }

    DPRINTF(VNC, "Sending security auth OK\n");

    uint32_t success = htobe(VncOK);
    write(&success);
    curState = WaitForClientInit;
}

void
VncServer::sendServerInit()
{
    ServerInitMsg msg;

    DPRINTF(VNC, "Sending server init message to client\n");

    msg.fbWidth = htobe(videoWidth());
    msg.fbHeight = htobe(videoHeight());

    msg.px.bpp = htobe(pixelFormat.bpp);
    msg.px.depth = htobe(pixelFormat.depth);
    msg.px.bigendian = htobe(pixelFormat.bigendian);
    msg.px.truecolor = htobe(pixelFormat.truecolor);
    msg.px.redmax = htobe(pixelFormat.redmax);
    msg.px.greenmax = htobe(pixelFormat.greenmax);
    msg.px.bluemax = htobe(pixelFormat.bluemax);
    msg.px.redshift = htobe(pixelFormat.redshift);
    msg.px.greenshift = htobe(pixelFormat.greenshift);
    msg.px.blueshift = htobe(pixelFormat.blueshift);
    memset(msg.px.padding, 0, 3);
    msg.namelen = 2;
    msg.namelen = htobe(msg.namelen);
    memcpy(msg.name, "M5", 2);

    write(&msg);
    curState = NormalPhase;
}

void
VncServer::setPixelFormat()
{
    DPRINTF(VNC, "Received pixel format from client message\n");

    PixelFormatMessage pfm;
    read1((uint8_t*)&pfm, sizeof(PixelFormatMessage));

    DPRINTF(VNC, " -- bpp = %d; depth = %d; be = %d\n", pfm.px.bpp,
            pfm.px.depth, pfm.px.bigendian);
    DPRINTF(VNC, " -- true color = %d red,green,blue max = %d,%d,%d\n",
            pfm.px.truecolor, betoh(pfm.px.redmax), betoh(pfm.px.greenmax),
                betoh(pfm.px.bluemax));
    DPRINTF(VNC, " -- red,green,blue shift = %d,%d,%d\n", pfm.px.redshift,
            pfm.px.greenshift, pfm.px.blueshift);

    if (betoh(pfm.px.bpp) != pixelFormat.bpp ||
        betoh(pfm.px.depth) != pixelFormat.depth ||
        betoh(pfm.px.bigendian) != pixelFormat.bigendian ||
        betoh(pfm.px.truecolor) != pixelFormat.truecolor ||
        betoh(pfm.px.redmax) != pixelFormat.redmax ||
        betoh(pfm.px.greenmax) != pixelFormat.greenmax ||
        betoh(pfm.px.bluemax) != pixelFormat.bluemax ||
        betoh(pfm.px.redshift) != pixelFormat.redshift ||
        betoh(pfm.px.greenshift) != pixelFormat.greenshift ||
        betoh(pfm.px.blueshift) != pixelFormat.blueshift)
        fatal("VNC client doesn't support true color raw encoding\n");
}

void
VncServer::setEncodings()
{
    DPRINTF(VNC, "Received supported encodings from client\n");

    PixelEncodingsMessage pem;
    read1((uint8_t*)&pem, sizeof(PixelEncodingsMessage));

    pem.num_encodings = betoh(pem.num_encodings);

    DPRINTF(VNC, " -- %d encoding present\n", pem.num_encodings);
    supportsRawEnc = supportsResizeEnc = false;

    for (int x = 0; x < pem.num_encodings; x++) {
        int32_t encoding;
        size_t len M5_VAR_USED;
        len = read(&encoding);
        assert(len == sizeof(encoding));
        DPRINTF(VNC, " -- supports %d\n", betoh(encoding));

        switch (betoh(encoding)) {
          case EncodingRaw:
            supportsRawEnc = true;
            break;
          case EncodingDesktopSize:
            supportsResizeEnc = true;
            break;
        }
    }

    if (!supportsRawEnc)
        fatal("VNC clients must always support raw encoding\n");
}

void
VncServer::requestFbUpdate()
{
    DPRINTF(VNC, "Received frame buffer update request from client\n");

    FrameBufferUpdateReq fbr;
    read1((uint8_t*)&fbr, sizeof(FrameBufferUpdateReq));

    fbr.x = betoh(fbr.x);
    fbr.y = betoh(fbr.y);
    fbr.width = betoh(fbr.width);
    fbr.height = betoh(fbr.height);

    DPRINTF(VNC, " -- x = %d y = %d w = %d h = %d\n", fbr.x, fbr.y, fbr.width,
            fbr.height);

    sendFrameBufferUpdate();
}

void
VncServer::recvKeyboardInput()
{
    DPRINTF(VNC, "Received keyboard input from client\n");
    KeyEventMessage kem;
    read1((uint8_t*)&kem, sizeof(KeyEventMessage));

    kem.key = betoh(kem.key);
    DPRINTF(VNC, " -- received key code %d (%s)\n", kem.key, kem.down_flag ?
            "down" : "up");

    if (keyboard)
        keyboard->keyPress(kem.key, kem.down_flag);
}

void
VncServer::recvPointerInput()
{
    DPRINTF(VNC, "Received pointer input from client\n");
    PointerEventMessage pem;

    read1((uint8_t*)&pem, sizeof(PointerEventMessage));;

    pem.x = betoh(pem.x);
    pem.y = betoh(pem.y);
    DPRINTF(VNC, " -- pointer at x = %d y = %d buttons = %#x\n", pem.x, pem.y,
            pem.button_mask);

    if (mouse)
        mouse->mouseAt(pem.x, pem.y, pem.button_mask);
}

void
VncServer::recvCutText()
{
    DPRINTF(VNC, "Received client copy buffer message\n");

    ClientCutTextMessage cct;
    read1((uint8_t*)&cct, sizeof(ClientCutTextMessage));

    char str[1025];
    size_t data_len = betoh(cct.length);
    DPRINTF(VNC, "String length %d\n", data_len);
    while (data_len > 0) {
        size_t len;
        size_t bytes_to_read = data_len > 1024 ? 1024 : data_len;
        len = read((uint8_t*)&str, bytes_to_read);
        str[bytes_to_read] = 0;
        assert(len >= data_len);
        data_len -= len;
        DPRINTF(VNC, "Buffer: %s\n", str);
    }

}


void
VncServer::sendFrameBufferUpdate()
{

    if (dataFd <= 0 || curState != NormalPhase || !sendUpdate) {
        DPRINTF(VNC, "NOT sending framebuffer update\n");
        return;
    }

    // The client will request data constantly, unless we throttle it
    sendUpdate = false;

    DPRINTF(VNC, "Sending framebuffer update\n");

    FrameBufferUpdate fbu;
    FrameBufferRect fbr;

    fbu.type = ServerFrameBufferUpdate;
    fbu.num_rects = 1;
    fbr.x = 0;
    fbr.y = 0;
    fbr.width = videoWidth();
    fbr.height = videoHeight();
    fbr.encoding = EncodingRaw;

    // fix up endian
    fbu.num_rects = htobe(fbu.num_rects);
    fbr.x = htobe(fbr.x);
    fbr.y = htobe(fbr.y);
    fbr.width = htobe(fbr.width);
    fbr.height = htobe(fbr.height);
    fbr.encoding = htobe(fbr.encoding);

    // send headers to client
    write(&fbu);
    write(&fbr);

    assert(fb);

    std::vector<uint8_t> line_buffer(pixelConverter.length * fb->width());
    for (int y = 0; y < fb->height(); ++y) {
        // Convert and send a line at a time
        uint8_t *raw_pixel(line_buffer.data());
        for (unsigned x = 0; x < fb->width(); ++x) {
            pixelConverter.fromPixel(raw_pixel, fb->pixel(x, y));
            raw_pixel += pixelConverter.length;
        }

        write(line_buffer.data(), line_buffer.size());
    }
}

void
VncServer::sendFrameBufferResized()
{
    assert(fb && dataFd > 0 && curState == NormalPhase);
    DPRINTF(VNC, "Sending framebuffer resize\n");

    FrameBufferUpdate fbu;
    FrameBufferRect fbr;

    fbu.type = ServerFrameBufferUpdate;
    fbu.num_rects = 1;
    fbr.x = 0;
    fbr.y = 0;
    fbr.width = videoWidth();
    fbr.height = videoHeight();
    fbr.encoding = EncodingDesktopSize;

    // fix up endian
    fbu.num_rects = htobe(fbu.num_rects);
    fbr.x = htobe(fbr.x);
    fbr.y = htobe(fbr.y);
    fbr.width = htobe(fbr.width);
    fbr.height = htobe(fbr.height);
    fbr.encoding = htobe(fbr.encoding);

    // send headers to client
    write(&fbu);
    write(&fbr);

    // No actual data is sent in this message
}

void
VncServer::setDirty()
{
    VncInput::setDirty();

    sendUpdate = true;
    sendFrameBufferUpdate();
}

void
VncServer::frameBufferResized()
{
    if (dataFd > 0 && curState == NormalPhase) {
        if (supportsResizeEnc)
            sendFrameBufferResized();
        else
            // The frame buffer changed size and we can't update the client
            detach();
    }
}

// create the VNC server object
VncServer *
VncServerParams::create()
{
    return new VncServer(this);
}

