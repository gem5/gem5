/*
 * Copyright (c) 2010-2013, 2015, 2017 ARM Limited
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
 * Implementiation of the ARM HDLcd controller.
 *
 * This implementation aims to have sufficient detail such that underrun
 * conditions are reasonable / behave similar to reality.  There are two
 * 'engines' going at once.  First, the DMA engine running at LCD clock
 * frequency is responsible for filling the controller's internal buffer.
 * The second engine runs at the pixel clock frequency and reads the pixels
 * out of the internal buffer.  The pixel rendering engine uses front / back
 * porch and sync delays between lines and frames.
 *
 * If the pixel rendering engine does not have a pixel to display, it will
 * cause an underrun event.  The HDLcd controller, per spec, will stop
 * issuing DMA requests for the rest of the frame and resume normal behavior
 * on the subsequent frame.  What pixels are rendered upon an underrun
 * condition is different than the real hardware; while the user will see
 * artifacts (previous frame mixed with current frame), it is not the same
 * behavior as real hardware which repeats the last pixel value for the rest
 * of the current frame.  This compromise was made to save on memory and
 * complexity and assumes that it is not important to accurately model the
 * content of an underrun frame.
 *
 * KNOWN ISSUES
 * <ul>
 *   <li>The HDLcd is implemented here as an AmbaDmaDevice, but it
 *       doesn't have an AMBA ID as far as I know.  That is the only
 *       bit of the AmbaDmaDevice interface that is irrelevant to it,
 *       so a fake AMBA ID is used for now.  I didn't think inserting
 *       an extra layer of hierachy between AmbaDmaDevice and
 *       DmaDevice would be helpful to anyone else, but that may be
 *       the right answer.
 * </ul>
 */

#ifndef __DEV_ARM_HDLCD_HH__
#define __DEV_ARM_HDLCD_HH__

#include <fstream>
#include <memory>
#include <vector>

#include "base/framebuffer.hh"
#include "base/imgwriter.hh"
#include "base/output.hh"
#include "dev/arm/amba_device.hh"
#include "dev/pixelpump.hh"
#include "sim/serialize.hh"

namespace gem5
{

class VncInput;
struct HDLcdParams;
class HDLcdPixelPump;

class HDLcd: public AmbaDmaDevice
{
  public:
    HDLcd(const HDLcdParams &p);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    void drainResume() override;

  public: // IO device interface
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    AddrRangeList getAddrRanges() const override { return addrRanges; }

  protected: // Parameters
    VncInput *vnc;
    const bool workaroundSwapRB;
    const bool workaroundDmaLineCount;
    const AddrRangeList addrRanges;
    const bool enableCapture;
    const Addr pixelBufferSize;
    const Tick virtRefreshRate;

  protected: // Register handling
    /** ARM HDLcd register offsets */
    enum RegisterOffset
    {
        Version          = 0x0000,
        Int_RawStat      = 0x0010,
        Int_Clear        = 0x0014,
        Int_Mask         = 0x0018,
        Int_Status       = 0x001C,
        Fb_Base          = 0x0100,
        Fb_Line_Length   = 0x0104,
        Fb_Line_Count    = 0x0108,
        Fb_Line_Pitch    = 0x010C,
        Bus_Options      = 0x0110,
        V_Sync           = 0x0200,
        V_Back_Porch     = 0x0204,
        V_Data           = 0x0208,
        V_Front_Porch    = 0x020C,
        H_Sync           = 0x0210,
        H_Back_Porch     = 0x0214,
        H_Data           = 0x0218,
        H_Front_Porch    = 0x021C,
        Polarities       = 0x0220,
        Command          = 0x0230,
        Pixel_Format     = 0x0240,
        Red_Select       = 0x0244,
        Green_Select     = 0x0248,
        Blue_Select      = 0x024C,
    };

    /** Reset value for Bus_Options register */
    static constexpr size_t BUS_OPTIONS_RESETV = 0x408;

    /** Reset value for Version register */
    static constexpr size_t VERSION_RESETV = 0x1CDC0000;

    /** AXI port width in bytes */
    static constexpr size_t AXI_PORT_WIDTH = 8;

    /** max number of beats delivered in one dma burst */
    static constexpr size_t MAX_BURST_LEN = 16;

    /** Maximum number of bytes per pixel */
    static constexpr size_t MAX_PIXEL_SIZE = 4;

    /**
     * @name RegisterFieldLayouts
     * Bit layout declarations for multi-field registers.
     */
    /**@{*/
    BitUnion32(VersionReg)
        Bitfield<7,0>   version_minor;
        Bitfield<15,8>  version_major;
        Bitfield<31,16> product_id;
    EndBitUnion(VersionReg)

    static constexpr uint32_t INT_DMA_END = (1UL << 0);
    static constexpr uint32_t INT_BUS_ERROR = (1UL << 1);
    static constexpr uint32_t INT_VSYNC = (1UL << 2);
    static constexpr uint32_t INT_UNDERRUN = (1UL << 3);

    BitUnion32(FbLineCountReg)
        Bitfield<11,0>  fb_line_count;
        Bitfield<31,12> reserved_31_12;
    EndBitUnion(FbLineCountReg)

    BitUnion32(BusOptsReg)
        Bitfield<4,0>   burst_len;
        Bitfield<7,5>   reserved_7_5;
        Bitfield<11,8>  max_outstanding;
        Bitfield<31,12> reserved_31_12;
    EndBitUnion(BusOptsReg)

    BitUnion32(TimingReg)
        Bitfield<11,0>  val;
        Bitfield<31,12> reserved_31_12;
    EndBitUnion(TimingReg)

    BitUnion32(PolaritiesReg)
        Bitfield<0>    vsync_polarity;
        Bitfield<1>    hsync_polarity;
        Bitfield<2>    dataen_polarity;
        Bitfield<3>    data_polarity;
        Bitfield<4>    pxlclk_polarity;
        Bitfield<31,5> reserved_31_5;
    EndBitUnion(PolaritiesReg)

    BitUnion32(CommandReg)
        Bitfield<0>    enable;
        Bitfield<31,1> reserved_31_1;
    EndBitUnion(CommandReg)

    BitUnion32(PixelFormatReg)
        Bitfield<2,0>  reserved_2_0;
        Bitfield<4,3>  bytes_per_pixel;
        Bitfield<30,5> reserved_30_5;
        Bitfield<31>   big_endian;
    EndBitUnion(PixelFormatReg)

    BitUnion32(ColorSelectReg)
        Bitfield<4,0>   offset;
        Bitfield<7,5>   reserved_7_5;
        Bitfield<11,8>  size;
        Bitfield<15,12> reserved_15_12;
        Bitfield<23,16> default_color;
        Bitfield<31,24> reserved_31_24;
    EndBitUnion(ColorSelectReg)
    /**@}*/

    /**
     * @name HDLCDRegisters
     * HDLCD register contents.
     */
    /**@{*/
    const VersionReg version = VERSION_RESETV;
                                    /**< Version register */
    uint32_t int_rawstat = 0;       /**< Interrupt raw status register */
    uint32_t int_mask = 0;          /**< Interrupt mask register */
    uint32_t fb_base = 0;           /**< Frame buffer base address register */
    uint32_t fb_line_length = 0;    /**< Frame buffer Line length register */
                                    /**< Frame buffer Line count register */
    FbLineCountReg fb_line_count = 0;
    int32_t fb_line_pitch = 0;      /**< Frame buffer Line pitch register */
    BusOptsReg bus_options = BUS_OPTIONS_RESETV;
                                    /**< Bus options register */
    TimingReg v_sync = 0;           /**< Vertical sync width register */
    TimingReg v_back_porch = 0;     /**< Vertical back porch width register */
    TimingReg v_data = 0;           /**< Vertical data width register */
    TimingReg v_front_porch = 0;    /**< Vertical front porch width register */
    TimingReg h_sync = 0;           /**< Horizontal sync width register */
    TimingReg h_back_porch = 0;     /**< Horizontal back porch width reg */
    TimingReg h_data = 0;           /**< Horizontal data width register */
    TimingReg h_front_porch = 0;    /**< Horizontal front porch width reg */
    PolaritiesReg polarities = 0;   /**< Polarities register */
    CommandReg command = 0;         /**< Command register */
    PixelFormatReg pixel_format = 0;/**< Pixel format register */
    ColorSelectReg red_select = 0;  /**< Red color select register */
    ColorSelectReg green_select = 0;/**< Green color select register */
    ColorSelectReg blue_select = 0; /**< Blue color select register */
    /** @} */

    std::vector<uint8_t> lineBuffer;

    uint32_t readReg(Addr offset);
    void writeReg(Addr offset, uint32_t value);

    PixelConverter pixelConverter() const;
    DisplayTimings displayTimings() const;

    void createDmaEngine();

    void cmdEnable();
    void cmdDisable();

    bool enabled() const { return command.enable; }

  public: // Pixel pump callbacks
    bool pxlNext(Pixel &p);
    size_t lineNext(std::vector<Pixel>::iterator pixel_it, size_t line_length);
    void pxlVSyncBegin();
    void pxlVSyncEnd();
    void pxlUnderrun();
    void pxlFrameDone();

  protected: // Interrupt handling
    /**
     * Assign new interrupt values and update interrupt signals
     *
     * A new interrupt is scheduled signalled if the set of unmasked
     * interrupts goes empty to non-empty. Conversely, if the set of
     * unmasked interrupts goes from non-empty to empty, the interrupt
     * signal is cleared.
     *
     * @param ints New <i>raw</i> interrupt status
     * @param mask New interrupt mask
     */
    void setInterrupts(uint32_t ints, uint32_t mask);

    /**
     * Convenience function to update the interrupt mask
     *
     * @see setInterrupts
     * @param mask New interrupt mask
     */
    void intMask(uint32_t mask) { setInterrupts(int_rawstat, mask); }

    /**
     * Convenience function to raise a new interrupt
     *
     * @see setInterrupts
     * @param ints Set of interrupts to raise
     */
    void
    intRaise(uint32_t ints)
    {
        setInterrupts(int_rawstat | ints, int_mask);
    }

    /**
     * Convenience function to clear interrupts
     *
     * @see setInterrupts
     * @param ints Set of interrupts to clear
     */
    void
    intClear(uint32_t ints)
    {
        setInterrupts(int_rawstat & ~ints, int_mask);
    }

    /** Masked interrupt status register */
    uint32_t intStatus() const { return int_rawstat & int_mask; }

  protected: // Pixel output
    class PixelPump : public BasePixelPump
    {
      public:
        PixelPump(HDLcd &p, ClockDomain &pxl_clk, unsigned pixel_chunk)
            : BasePixelPump(p, pxl_clk, pixel_chunk), parent(p)
        {}

        void dumpSettings();

      protected:
        bool nextPixel(Pixel &p) override { return parent.pxlNext(p); }
        size_t
        nextLine(std::vector<Pixel>::iterator pixel_it,
                 size_t line_length) override
        {
            return parent.lineNext(pixel_it, line_length);
        }

        void onVSyncBegin() override { return parent.pxlVSyncBegin(); }
        void onVSyncEnd() override { return parent.pxlVSyncEnd(); }

        void
        onUnderrun(unsigned x, unsigned y) override
        {
            parent.pxlUnderrun();
        }

        void onFrameDone() override { parent.pxlFrameDone(); }

      protected:
        HDLcd &parent;
    };

    Addr bypassLineAddress = 0;

    /** Handler for fast frame refresh in KVM-mode */
    void virtRefresh();
    EventFunctionWrapper virtRefreshEvent;

    /** Helper to write out bitmaps */
    std::unique_ptr<ImgWriter> imgWriter;

    /** Image Format */
    enums::ImageFormat imgFormat;

    /** Picture of what the current frame buffer looks like */
    OutputStream *pic = nullptr;

    /** Cached pixel converter, set when the converter is enabled. */
    PixelConverter conv = PixelConverter::rgba8888_le;

    PixelPump pixelPump;

  protected: // DMA handling
    class DmaEngine : public DmaReadFifo
    {
      public:
        DmaEngine(HDLcd &_parent, size_t size,
                  unsigned request_size, unsigned max_pending,
                  size_t line_size, ssize_t line_pitch, unsigned num_lines);

        void startFrame(Addr fb_base);
        void abortFrame();
        void dumpSettings();

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

      protected:
        void onEndOfBlock() override;
        void onIdle() override;

        HDLcd &parent;
        const size_t lineSize;
        const ssize_t linePitch;
        const unsigned numLines;

        Addr nextLineAddr;
        Addr frameEnd;
    };

    std::unique_ptr<DmaEngine> dmaEngine;

  protected: // Statistics
    struct HDLcdStats: public statistics::Group
    {
        HDLcdStats(statistics::Group *parent);
        statistics::Scalar underruns;
    } stats;
};

} // namespace gem5

#endif
