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
 * Authors: William Wang
 *          Ali Saidi
 */


/** @file
 * Implementiation of a PL111 CLCD controller
 */

#ifndef __DEV_ARM_PL111_HH__
#define __DEV_ARM_PL111_HH__

#include <fstream>

#include "base/range.hh"
#include "dev/arm/amba_device.hh"
#include "params/Pl111.hh"
#include "sim/serialize.hh"

class Gic;
class VncServer;
class Bitmap;

class Pl111: public AmbaDmaDevice
{
  protected:
    static const uint64_t AMBA_ID       = ULL(0xb105f00d00141111);
    /** ARM PL111 register map*/
    static const int LcdTiming0       = 0x000;
    static const int LcdTiming1       = 0x004;
    static const int LcdTiming2       = 0x008;
    static const int LcdTiming3       = 0x00C;
    static const int LcdUpBase        = 0x010;
    static const int LcdLpBase        = 0x014;
    static const int LcdControl       = 0x018;
    static const int LcdImsc          = 0x01C;
    static const int LcdRis           = 0x020;
    static const int LcdMis           = 0x024;
    static const int LcdIcr           = 0x028;
    static const int LcdUpCurr        = 0x02C;
    static const int LcdLpCurr        = 0x030;
    static const int LcdPalette       = 0x200;
    static const int CrsrImage        = 0x800;
    static const int ClcdCrsrCtrl     = 0xC00;
    static const int ClcdCrsrConfig   = 0xC04;
    static const int ClcdCrsrPalette0 = 0xC08;
    static const int ClcdCrsrPalette1 = 0xC0C;
    static const int ClcdCrsrXY       = 0xC10;
    static const int ClcdCrsrClip     = 0xC14;
    static const int ClcdCrsrImsc     = 0xC20;
    static const int ClcdCrsrIcr      = 0xC24;
    static const int ClcdCrsrRis      = 0xC28;
    static const int ClcdCrsrMis      = 0xC2C;

    static const int LcdPaletteSize   = 128;
    static const int CrsrImageSize    = 256;

    static const int LcdMaxWidth      = 1024; // pixels per line
    static const int LcdMaxHeight     = 768;  // lines per panel

    static const int dmaSize            = 8;    // 64 bits
    static const int maxOutstandingDma  = 16;   // 16 deep FIFO of 64 bits

    enum LcdMode {
        bpp1 = 0,
        bpp2,
        bpp4,
        bpp8,
        bpp16,
        bpp24,
        bpp16m565,
        bpp12
    };

    BitUnion8(InterruptReg)
        Bitfield<1> underflow;
        Bitfield<2> baseaddr;
        Bitfield<3> vcomp;
        Bitfield<4> ahbmaster;
    EndBitUnion(InterruptReg)

    BitUnion32(TimingReg0)
        Bitfield<7,2> ppl;
        Bitfield<15,8> hsw;
        Bitfield<23,16> hfp;
        Bitfield<31,24> hbp;
    EndBitUnion(TimingReg0)

    BitUnion32(TimingReg1)
        Bitfield<9,0> lpp;
        Bitfield<15,10> vsw;
        Bitfield<23,16> vfp;
        Bitfield<31,24> vbp;
    EndBitUnion(TimingReg1)

    BitUnion32(TimingReg2)
        Bitfield<4,0> pcdlo;
        Bitfield<5> clksel;
        Bitfield<10,6> acb;
        Bitfield<11> avs;
        Bitfield<12> ihs;
        Bitfield<13> ipc;
        Bitfield<14> ioe;
        Bitfield<25,16> cpl;
        Bitfield<26> bcd;
        Bitfield<31,27> pcdhi;
    EndBitUnion(TimingReg2)

    BitUnion32(TimingReg3)
        Bitfield<6,0> led;
        Bitfield<16> lee;
    EndBitUnion(TimingReg3)

    BitUnion32(ControlReg)
        Bitfield<0> lcden;
        Bitfield<3,1> lcdbpp;
        Bitfield<4> lcdbw;
        Bitfield<5> lcdtft;
        Bitfield<6> lcdmono8;
        Bitfield<7> lcddual;
        Bitfield<8> bgr;
        Bitfield<9> bebo;
        Bitfield<10> bepo;
        Bitfield<11> lcdpwr;
        Bitfield<13,12> lcdvcomp;
        Bitfield<16> watermark;
    EndBitUnion(ControlReg)

    /** Horizontal axis panel control register */
    TimingReg0 lcdTiming0;

    /** Vertical axis panel control register */
    TimingReg1 lcdTiming1;

    /** Clock and signal polarity control register */
    TimingReg2 lcdTiming2;

    /** Line end control register */
    TimingReg3 lcdTiming3;

    /** Upper panel frame base address register */
    int lcdUpbase;

    /** Lower panel frame base address register */
    int lcdLpbase;

    /** Control register */
    ControlReg lcdControl;

    /** Interrupt mask set/clear register */
    InterruptReg lcdImsc;

    /** Raw interrupt status register - const */
    InterruptReg lcdRis;

    /** Masked interrupt status register */
    InterruptReg lcdMis;

    /** 256x16-bit color palette registers
     * 256 palette entries organized as 128 locations of two entries per word */
    int lcdPalette[LcdPaletteSize];

    /** Cursor image RAM register
     * 256-word wide values defining images overlaid by the hw cursor mechanism */
    int cursorImage[CrsrImageSize];

    /** Cursor control register */
    int clcdCrsrCtrl;

    /** Cursor configuration register */
    int clcdCrsrConfig;

    /** Cursor palette registers */
    int clcdCrsrPalette0;
    int clcdCrsrPalette1;

    /** Cursor XY position register */
    int clcdCrsrXY;

    /** Cursor clip position register */
    int clcdCrsrClip;

    /** Cursor interrupt mask set/clear register */
    InterruptReg clcdCrsrImsc;

    /** Cursor interrupt clear register */
    InterruptReg clcdCrsrIcr;

    /** Cursor raw interrupt status register - const */
    InterruptReg clcdCrsrRis;

    /** Cursor masked interrupt status register - const */
    InterruptReg clcdCrsrMis;

    /** Clock speed */
    Tick clock;

    /** VNC server */
    VncServer *vncserver;

    /** Helper to write out bitmaps */
    Bitmap *bmp;

    /** Picture of what the current frame buffer looks like */
    std::ostream *pic;

    /** Frame buffer width - pixels per line */
    uint16_t width;

    /** Frame buffer height - lines per panel */
    uint16_t height;

    /** Bytes per pixel */
    uint8_t bytesPerPixel;

    /** CLCDC supports up to 1024x768 */
    uint8_t *dmaBuffer;

    /** Start time for frame buffer dma read */
    Tick startTime;

    /** Frame buffer base address */
    Addr startAddr;

    /** Frame buffer max address */
    Addr maxAddr;

    /** Frame buffer current address */
    Addr curAddr;

    /** DMA FIFO watermark */
    int waterMark;

    /** Number of pending dma reads */
    int dmaPendingNum;

    /** Send updated parameters to the vnc server */
    void updateVideoParams();

    /** DMA framebuffer read */
    void readFramebuffer();

    /** Generate dma framebuffer read event */
    void generateReadEvent();

    /** Function to generate interrupt */
    void generateInterrupt();

    /** fillFIFO event */
    void fillFifo();

    /** start the dmas off after power is enabled */
    void startDma();

    /** DMA done event */
    void dmaDone();

    /** Next cycle event */
    Tick nextCycle();
    Tick nextCycle(Tick beginTick);

    /** DMA framebuffer read event */
    EventWrapper<Pl111, &Pl111::readFramebuffer> readEvent;

    /** Fill fifo */
    EventWrapper<Pl111, &Pl111::fillFifo> fillFifoEvent;

    /** DMA done event */
    std::vector<EventWrapper<Pl111, &Pl111::dmaDone> > dmaDoneEvent;

    /** Wrapper to create an event out of the interrupt */
    EventWrapper<Pl111, &Pl111::generateInterrupt> intEvent;

  public:
    typedef Pl111Params Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    Pl111(const Params *p);

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    /**
     * Determine the address ranges that this device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges();
};

#endif
