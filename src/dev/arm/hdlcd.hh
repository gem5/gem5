/*
 * Copyright (c) 2010-2013 ARM Limited
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
 * Authors: Chris Emmons
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
 * 1.  The default kernel driver used in testing sets the line count to one
 * less than the expected 768.  However, it also sets the v_count to 767.
 * The controller specifies that 1 should be added to v_count but does not
 * specify adding 1 to the line count.  The driver is probably wrong.
 * However, to sync these two numbers up, this model uses fb_line_count and
 * fb_line_length rather than using v_data or h_data values to determine the
 * width and height of the frame; those values are ignored.
 * 2.  The HDLcd is implemented here as an AmbaDmaDevice, but it doesn't have
 * an AMBA ID as far as I know.  That is the only bit of the AmbaDmaDevice
 * interface that is irrelevant to it, so a fake AMBA ID is used for now.
 * I didn't think inserting an extra layer of hierachy between AmbaDmaDevice
 * and DmaDevice would be helpful to anyone else, but that may be the right
 * answer.
 * 3.  The internal buffer size is either 1 or 2 KB depending on which
 * specification is referenced for the different Versatile Express tiles.
 * This implementation uses the larger 2 KB buffer by default.
 */

#ifndef __DEV_ARM_HDLCD_HH__
#define __DEV_ARM_HDLCD_HH__

#include <fstream>

#include "dev/arm/amba_device.hh"
#include "params/HDLcd.hh"
#include "sim/serialize.hh"

class VncInput;
class Bitmap;

class HDLcd: public AmbaDmaDevice
{
  protected:
    /** fake AMBA ID -- unused */
    static const uint64_t AMBA_ID       = ULL(0xb105f00d00141000);

    /** ARM HDLcd register offsets */
    enum RegisterOffset {
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
        Blue_Select      = 0x024C };

    /** Reset value for Bus_Options register */
    static const size_t BUS_OPTIONS_RESETV = 0x408;

    /** Reset value for Version register */
    static const size_t VERSION_RESETV = 0x1CDC0000;

    /** max number of outstanding DMA requests possible */
    static const size_t MAX_OUTSTANDING_DMA_REQ_CAPACITY = 16;

    /** max number of beats delivered in one dma burst */
    static const size_t MAX_BURST_LEN = 16;

    /** size of internal buffer in bytes */
    static const size_t PIXEL_BUFFER_CAPACITY = 2048;

    /** AXI port width in bytes */
    static const size_t AXI_PORT_WIDTH = 8;

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

    BitUnion32(InterruptReg)
        Bitfield<0> dma_end;
        Bitfield<1> bus_error;
        Bitfield<2> vsync;
        Bitfield<3> underrun;
    EndBitUnion(InterruptReg)

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
    VersionReg version;             /**< Version register */
    InterruptReg int_rawstat;       /**< Interrupt raw status register */
    InterruptReg int_clear;         /**< Interrupt clear register */
    InterruptReg int_mask;          /**< Interrupt mask register */
    InterruptReg int_status;        /**< Interrupt status register */
    uint32_t fb_base;               /**< Frame buffer base address register */
    uint32_t fb_line_length;        /**< Frame buffer Line length register */
    FbLineCountReg fb_line_count;   /**< Frame buffer Line count register */
    uint32_t fb_line_pitch;         /**< Frame buffer Line pitch register */
    BusOptsReg bus_options;         /**< Bus options register */
    TimingReg v_sync;               /**< Vertical sync width register */
    TimingReg v_back_porch;         /**< Vertical back porch width register */
    TimingReg v_data;               /**< Vertical data width register */
    TimingReg v_front_porch;        /**< Vertical front porch width register */
    TimingReg h_sync;               /**< Horizontal sync width register */
    TimingReg h_back_porch;         /**< Horizontal back porch width register */
    TimingReg h_data;               /**< Horizontal data width register */
    TimingReg h_front_porch;        /**< Horizontal front porch width reg */
    PolaritiesReg polarities;       /**< Polarities register */
    CommandReg command;             /**< Command register */
    PixelFormatReg pixel_format;    /**< Pixel format register */
    ColorSelectReg red_select;      /**< Red color select register */
    ColorSelectReg green_select;    /**< Green color select register */
    ColorSelectReg blue_select;     /**< Blue color select register */
    /** @} */

    /** Pixel clock period */
    const Tick pixelClock;

    /** VNC server */
    VncInput *vnc;

    /** Helper to write out bitmaps */
    Bitmap *bmp;

    /** Picture of what the current frame buffer looks like */
    std::ostream *pic;

    /**
     * Event wrapper for dmaDone()
     *
     * This event call pushes its this pointer onto the freeDoneEvent vector
     * and calls dmaDone() when triggered.  While most of the time the burst
     * length of a transaction will be the max burst length set by the driver,
     * any trailing bytes must be handled with smaller lengths thus requiring
     * the configurable burst length option.
     */
    class DmaDoneEvent : public Event
    {
      private:
        /** Reference to HDLCD that issued the corresponding DMA transaction */
        HDLcd &obj;

        /** Transaction size */
        size_t transSize;

      public:
        /**
         * Constructor.
         *
         * @param _obj HDLCD that issued the corresponding DMA transaction
         */
        DmaDoneEvent(HDLcd *_obj)
            : Event(), obj(*_obj), transSize(0) {}

        /**
         * Sets the size of this transaction.
         *
         * @param len size of the transaction in bytes
         */
        void setTransactionSize(size_t len) {
            transSize = len;
        }

        /**
         * Gets the size of this transaction.
         *
         * @return size of this transaction in bytes
         */
        size_t getTransactionSize() const {
            return transSize;
        }

        void process() {
            obj.dmaDone(this);
        }

        const std::string name() const {
            return obj.name() + ".DmaDoneEvent";
        }
    };

    /** Start time for frame buffer dma read */
    Tick frameReadStartTime;

    /** Starting address for the current frame */
    Addr dmaStartAddr;

    /** Next address the dma should read from */
    Addr dmaCurAddr;

    /** One byte past the address of the last byte the dma should read
      * from */
    Addr dmaMaxAddr;

    /** Number of pending dma reads */
    size_t dmaPendingNum;

    /** Flag indicating whether current frame has underrun */
    bool frameUnderrun;

    /** HDLcd virtual display buffer */
    uint8_t *virtualDisplayBuffer;

    /** Size of the pixel buffer */
    size_t pixelBufferSize;

    /** Index of the next pixel to render */
    size_t pixelIndex;

    /** Flag indicating whether video parameters need updating */
    bool doUpdateParams;

    /** Flag indicating whether a frame read / display is in progress */
    bool frameUnderway;

    /**
     * Number of bytes in flight from DMA that have not reached the pixel
     * buffer yet
     */
    uint32_t dmaBytesInFlight;

    /**
     * Gets the number of oustanding DMA transactions allowed on the bus at a
     * time.
     *
     * @return gets the driver-specified number of outstanding DMA transactions
     *         from the hdlcd controller that are allowed on the bus at a time
     */
    inline uint16_t maxOutstandingDma() const {
        return bus_options.max_outstanding;
    }

    /**
     * Gets the number of bytes free in the pixel buffer.
     *
     * @return number of bytes free in the internal pixel buffer
     */
    inline uint32_t bytesFreeInPixelBuffer() const {
        return PIXEL_BUFFER_CAPACITY - (pixelBufferSize + dmaBytesInFlight);
    }

    /**
     * Gets the number of beats-per-burst for bus transactions.
     *
     * @return number of beats-per-burst per HDLcd DMA transaction
     */
    inline size_t dmaBurstLength() const {
        assert(bus_options.burst_len <= MAX_BURST_LEN);
        return bus_options.burst_len;
    }

    /**
     * Gets the number of bytes per pixel.
     *
     * @return bytes per pixel
     */
    inline size_t bytesPerPixel() const {
        return pixel_format.bytes_per_pixel + 1;
    }

    /**
     * Gets frame buffer width.
     *
     * @return frame buffer width (pixels per line)
     */
    inline size_t width() const {
        return fb_line_length / bytesPerPixel();
    }

    /**
     * Gets frame buffer height.
     *
     * @return frame buffer height (lines per panel)
     */
    inline size_t height() const {
        return fb_line_count.fb_line_count;
    }

    /**
     * Gets the total number of pixel clocks per display line.
     *
     * @return number of pixel clocks per display line including porch delays
     *         and horizontal sync time
     */
    inline uint64_t PClksPerLine() const {
        return h_back_porch.val + 1 +
               h_data.val + 1 +
               h_front_porch.val + 1 +
               h_sync.val + 1;
    }

    /** Send updated parameters to the vnc server */
    void updateVideoParams(bool unserializing);

    /** Generates an interrupt */
    void generateInterrupt();

    /** Start reading the next frame */
    void startFrame();

    /** End of frame reached */
    void endFrame();

    /** Generate DMA read requests from frame buffer into pixel buffer */
    void fillPixelBuffer();

    /** DMA done event */
    void dmaDone(DmaDoneEvent *event);

    /** Called when it is time to render a pixel */
    void renderPixel();

    /** Start of frame event */
    EventWrapper<HDLcd, &HDLcd::startFrame> startFrameEvent;

    /** End of frame event */
    EventWrapper<HDLcd, &HDLcd::endFrame> endFrameEvent;

    /** Pixel render event */
    EventWrapper<HDLcd, &HDLcd::renderPixel> renderPixelEvent;

    /** Fill fifo */
    EventWrapper<HDLcd, &HDLcd::fillPixelBuffer> fillPixelBufferEvent;

    /** Wrapper to create an event out of the interrupt */
    EventWrapper<HDLcd, &HDLcd::generateInterrupt> intEvent;

    /**@{*/
    /**
     * All pre-allocated DMA done events
     *
     * The HDLCD model preallocates maxOutstandingDma() number of
     * DmaDoneEvents to avoid having to heap allocate every single
     * event when it is needed. In order to keep track of which events
     * are in flight and which are ready to be used, we use two
     * different vectors. dmaDoneEventAll contains <i>all</i>
     * DmaDoneEvents that the object may use, while dmaDoneEventFree
     * contains a list of currently <i>unused</i> events. When an
     * event needs to be scheduled, the last element of the
     * dmaDoneEventFree is used and removed from the list. When an
     * event fires, it is added to the end of the
     * dmaEventFreeList. dmaDoneEventAll is never used except for in
     * initialization and serialization.
     */
    std::vector<DmaDoneEvent> dmaDoneEventAll;

    /** Unused DMA done events that are ready to be scheduled */
    std::vector<DmaDoneEvent *> dmaDoneEventFree;
    /**@}*/

    bool enableCapture;

  public:
    typedef HDLcdParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
    HDLcd(const Params *p);
    ~HDLcd();

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);

    /**
     * Determine the address ranges that this device responds to.
     *
     * @return a list of non-overlapping address ranges
     */
    AddrRangeList getAddrRanges() const;
};

#endif
