/*
 * Copyright (c) 2015, 2017 ARM Limited
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

#ifndef __DEV_PIXELPUMP_HH__
#define __DEV_PIXELPUMP_HH__

#include <vector>

#include "base/framebuffer.hh"
#include "sim/clocked_object.hh"

namespace gem5
{

struct BasePixelPumpParams;

struct DisplayTimings : public Serializable
{
    /**
     * Create a display timing configuration struct
     *
     * @param width Width of the visible area of the screen.
     * @param height Height of the visible area of the screen.
     * @param hfp Horizontal front porch in pixel clocks.
     * @param h_sync Horizontal sync in pixel clocks.
     * @param hbp Horizontal back porch in pixel clocks.
     * @param vfp Vertical front porch in scan lines.
     * @param v_sync Vertical sync in scan lines.
     * @param vbp Vertical back porch in scan lines.
     */
    DisplayTimings(unsigned width, unsigned height, unsigned hbp,
                   unsigned h_sync, unsigned hfp, unsigned vbp,
                   unsigned v_sync, unsigned vfp);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /** How many pixel clocks are required for one line? */
    Cycles
    cyclesPerLine() const
    {
        return Cycles(hSync + hBackPorch + width + hBackPorch);
    }

    /** How many pixel clocks are required for one frame? */
    Cycles
    cyclesPerFrame() const
    {
        return Cycles(cyclesPerLine() * linesPerFrame());
    }

    /** Calculate the first line of the vsync signal */
    unsigned
    lineVSyncStart() const
    {
        return 0;
    }

    /** Calculate the first line of the vertical back porch */
    unsigned
    lineVBackPorchStart() const
    {
        return lineVSyncStart() + vSync;
    }

    /** Calculate the first line of the visible region */
    unsigned
    lineFirstVisible() const
    {
        return lineVBackPorchStart() + vBackPorch;
    }

    /** Calculate the first line of the back porch */
    unsigned
    lineFrontPorchStart() const
    {
        return lineFirstVisible() + height;
    }

    /** Calculate the total number of lines in a frame */
    unsigned
    linesPerFrame() const
    {
        return lineFrontPorchStart() + vFrontPorch;
    }

    /** Display width in pixels */
    unsigned width;
    /** Display height in pixels */
    unsigned height;

    /** Horizontal back porch in pixels */
    unsigned hBackPorch;
    /** Horizontal front porch in pixels */
    unsigned hFrontPorch;
    /** Horizontal sync signal length in pixels */
    unsigned hSync;

    /** Vertical back porch in lines */
    unsigned vBackPorch;
    /** Vertical front porch in lines */
    unsigned vFrontPorch;
    /** Vertical sync signal in lines */
    unsigned vSync;

    static const DisplayTimings vga;
};

/**
 * Timing generator for a pixel-based display.
 *
 * Pixels are ordered relative to the top left corner of the
 * display. Scan lines appear in the following order:
 * <ol>
 *   <li>Vertical Sync (starting at line 0)
 *   <li>Vertical back porch
 *   <li>Visible lines
 *   <li>Vertical front porch
 * </ol>
 *
 * Pixel order within a scan line:
 * <ol>
 *   <li>Horizontal Sync
 *   <li>Horizontal Back Porch
 *   <li>Visible pixels
 *   <li>Horizontal Front Porch
 * </ol>
 */
class BasePixelPump : public EventManager, public Clocked, public Serializable
{
  public:
    BasePixelPump(EventManager &em, ClockDomain &pxl_clk,
                  unsigned pixel_chunk);
    virtual ~BasePixelPump();

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public: // Public API
    /** Update frame size using display timing */
    void updateTimings(const DisplayTimings &timings);

    /** Render an entire frame in non-caching mode */
    void renderFrame();

    /** Starting pushing pixels in timing mode */
    void start();

    /** Immediately stop pushing pixels */
    void stop();

    /** Get a constant reference of the current display timings */
    const DisplayTimings &
    timings() const
    {
        return _timings;
    }

    /** Is the pixel pump active and refreshing the display? */
    bool
    active() const
    {
        return evBeginLine.active();
    }

    /** Did a buffer underrun occur within this refresh interval? */
    bool
    underrun() const
    {
        return _underrun;
    }

    /** Is the current line within the visible range? */
    bool
    visibleLine() const
    {
        return line >= _timings.lineFirstVisible() &&
               line < _timings.lineFrontPorchStart();
    }

    /** Current pixel position within the visible area */
    unsigned
    posX() const
    {
        return _posX;
    }

    /** Current pixel position within the visible area */
    unsigned
    posY() const
    {
        return visibleLine() ? line - _timings.lineFirstVisible() : 0;
    }

    /** Output frame buffer */
    FrameBuffer fb;

  protected: // Callbacks
    /**
     * Get the next pixel from the scan line buffer.
     *
     * @param p Output pixel value, undefined on underrun
     * @return true on success, false on buffer underrun
     */
    virtual bool nextPixel(Pixel &p) = 0;

    /**
     * Get the next line of pixels directly from memory. This is for use from
     * the renderFrame which is called in non-caching mode.
     *
     * The default implementation falls back to calling nextPixel over and
     * over, but a more efficient implementation could retrieve the entire line
     * of pixels all at once using fewer access to memory which bypass any
     * intermediate structures like an incoming FIFO.
     *
     * @param ps          A vector iterator to store retrieved pixels into.
     * @param line_length The number of pixels being requested.
     * @return The number of pixels actually retrieved.
     */
    virtual size_t
    nextLine(std::vector<Pixel>::iterator ps, size_t line_length)
    {
        size_t count = 0;
        while (count < line_length && nextPixel(*ps++))
            count++;
        return count;
    }

    /** First pixel clock of the first VSync line. */
    virtual void onVSyncBegin(){};

    /**
     * Callback on the first pixel of the line after the end VSync
     * region (typically the first pixel of the vertical back porch).
     */
    virtual void onVSyncEnd(){};

    /**
     * Start of the HSync region.
     *
     * @note This is called even for scan lines outside of the visible
     * region.
     */
    virtual void onHSyncBegin(){};

    /**
     * Start of the first pixel after the HSync region.
     *
     * @note This is called even for scan lines outside of the visible
     * region.
     */
    virtual void onHSyncEnd(){};

    /**
     * Buffer underrun occurred on a frame.
     *
     * This method is called once if there is buffer underrun while
     * refreshing the display. The underrun state is reset on the next
     * refresh.
     *
     * @param x Coordinate within the visible region.
     * @param y Coordinate within the visible region.
     */
    virtual void onUnderrun(unsigned x, unsigned y){};

    /** Finished displaying the visible region of a frame */
    virtual void onFrameDone(){};

  private: // Params
    /** Maximum number of pixels to handle per render callback */
    const unsigned pixelChunk;

  private:
    /**
     * Callback helper class with suspend support.
     *
     * Unlike a normal EventWrapper, this class suspends an event on
     * drain() and restarts it at drainResume(). The suspend operation
     * stores the tick relative to curTick() and then deschedules the
     * event. The resume operation schedules the event at curTick()
     * plus the relative tick stored when the event was suspended.
     */
    class PixelEvent : public Event, public Drainable
    {
        typedef void (BasePixelPump::*CallbackType)();

      public:
        PixelEvent(const char *name, BasePixelPump *parent, CallbackType func);

        DrainState drain() override;
        void drainResume() override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        const std::string
        name() const override
        {
            return _name;
        }

        void
        process() override
        {
            (parent.*func)();
        }

        bool
        active() const
        {
            return scheduled() || suspended;
        }

      private:
        void suspend();
        void resume();

        const std::string _name;
        BasePixelPump &parent;
        const CallbackType func;

        bool suspended;
        Tick relativeTick;
    };

    void beginLine();
    void renderPixels();

    /** Fast and event-free line rendering function */
    void renderLine();

    /** Convenience vector when doing operations on all events */
    std::vector<PixelEvent *> pixelEvents;

    PixelEvent evVSyncBegin;
    PixelEvent evVSyncEnd;
    PixelEvent evHSyncBegin;
    PixelEvent evHSyncEnd;
    PixelEvent evBeginLine;
    PixelEvent evRenderPixels;

    DisplayTimings _timings;

    /**
     * Current line (including back porch, front porch, and vsync)
     * within a frame.
     */
    unsigned line;
    /** X-coordinate within the visible region of a frame */
    unsigned _posX;

    /** Did a buffer underrun occur within this refresh interval? */
    bool _underrun;
};

} // namespace gem5

#endif // __DEV_PIXELPUMP_HH__
