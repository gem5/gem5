/*
 * Copyright (c) 2012-2013, 2017-2018, 2021 Arm Limited
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
 * Base class for ARM GIC implementations
 */

#ifndef __DEV_ARM_BASE_GIC_H__
#define __DEV_ARM_BASE_GIC_H__

#include <memory>
#include <unordered_map>
#include <vector>

#include "arch/arm/system.hh"
#include "dev/intpin.hh"
#include "dev/io_device.hh"

#include "enums/ArmInterruptType.hh"

namespace gem5
{

class Platform;
class RealView;
class ThreadContext;
class ArmInterruptPin;
class ArmSPI;
class ArmPPI;
class ArmSigInterruptPin;

struct ArmInterruptPinParams;
struct ArmPPIParams;
struct ArmSPIParams;
struct ArmSigInterruptPinParams;
struct BaseGicParams;

class BaseGic : public PioDevice
{
  public:
    typedef BaseGicParams Params;
    enum class GicVersion
    {
        GIC_V2,
        GIC_V3,
        GIC_V4
    };

    BaseGic(const Params &p);
    virtual ~BaseGic();
    void init() override;

    const Params &params() const;

    /**
     * Post an interrupt from a device that is connected to the GIC.
     *
     * Depending on the configuration, the GIC will pass this interrupt
     * on through to a CPU.
     *
     * @param num number of interrupt to send
     */
    virtual void sendInt(uint32_t num) = 0;

    /**
     * Interface call for private peripheral interrupts.
     *
     * @param num number of interrupt to send
     * @param cpu CPU to forward interrupt to
     */
    virtual void sendPPInt(uint32_t num, uint32_t cpu) = 0;
    virtual void clearPPInt(uint32_t num, uint32_t cpu) = 0;

    /**
     * Clear an interrupt from a device that is connected to the GIC.
     *
     * Depending on the configuration, the GIC may de-assert it's CPU
     * line.
     *
     * @param num number of interrupt to send
     */
    virtual void clearInt(uint32_t num) = 0;

    ArmSystem *
    getSystem() const
    {
        return (ArmSystem *)sys;
    }

    /** Check if version supported */
    virtual bool supportsVersion(GicVersion version) = 0;

  protected: // GIC state transfer
    /**
     * When trasferring the state between two GICs (essentially
     * writing architectural registers) an interrupt might be posted
     * by the model. We don't want this to happen as the GIC might
     * be in an inconsistent state. We therefore disable side effects
     * by relying on the blockIntUpdate method.
     */
    virtual bool
    blockIntUpdate() const
    {
        return false;
    }

  protected:
    /** Platform this GIC belongs to. */
    Platform *platform;
};

/**
 * This SimObject is instantiated in the python world and
 * serves as an ArmInterruptPin generator. In this way it
 * is possible to instantiate a single generator per component
 * during configuration, and to dynamically spawn ArmInterruptPins.
 * See ArmPPIGen for more info on how this is used.
 */
class ArmInterruptPinGen : public SimObject
{
  public:
    ArmInterruptPinGen(const ArmInterruptPinParams &p);

    virtual ArmInterruptPin *get(ThreadContext *tc = nullptr) = 0;
};

/**
 * Shared Peripheral Interrupt Generator
 * It is capable of generating one interrupt only: it maintains a pointer
 * to it and returns it every time it is asked for it (via the get metod)
 */
class ArmSPIGen : public ArmInterruptPinGen
{
  public:
    ArmSPIGen(const ArmSPIParams &p);

    ArmInterruptPin *get(ThreadContext *tc = nullptr) override;

  protected:
    ArmSPI *pin;
};

/**
 * Private Peripheral Interrupt Generator
 * Since PPIs are banked in the GIC, this class is capable of generating
 * more than one interrupt (one per ContextID).
 */
class ArmPPIGen : public ArmInterruptPinGen
{
  public:
    PARAMS(ArmPPI);
    ArmPPIGen(const Params &p);

    ArmInterruptPin *get(ThreadContext *tc = nullptr) override;

  protected:
    std::unordered_map<ContextID, ArmPPI *> pins;
};

class ArmSigInterruptPinGen : public ArmInterruptPinGen
{
  public:
    ArmSigInterruptPinGen(const ArmSigInterruptPinParams &p);

    ArmInterruptPin *get(ThreadContext *tc = nullptr) override;
    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

  protected:
    ArmSigInterruptPin *pin;
};

/**
 * Generic representation of an Arm interrupt pin.
 */
class ArmInterruptPin : public Serializable
{
    friend class ArmInterruptPinGen;

  protected:
    ArmInterruptPin(const ArmInterruptPinParams &p, ThreadContext *tc);

  public: /* Public interface */
    /**
     * Set the thread context owning this interrupt.
     *
     * This method is used to set the thread context for interrupts
     * that are thread/CPU-specific. Only devices that are used in
     * such a context are expected to call this method.
     */
    void setThreadContext(ThreadContext *tc);

    /** Get interrupt number */
    uint32_t
    num() const
    {
        return intNum;
    }

    /** True if interrupt pin is active, false otherwise */
    bool
    active() const
    {
        return _active;
    }

    /** Signal an interrupt */
    virtual void raise() = 0;
    /** Clear a signalled interrupt */
    virtual void clear() = 0;

  public: /* Serializable interface */
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  protected:
    /**
     * Get the target context ID of this interrupt.
     *
     * @pre setThreadContext() must have been called prior to calling
     * this method.
     */
    ContextID targetContext() const;

    /**
     * Pointer to the thread context that owns this interrupt in case
     * it is a thread-/CPU-private interrupt
     */
    const ThreadContext *threadContext;

    /** Arm platform to use for interrupt generation */
    RealView *const platform;

    /** Interrupt number to generate */
    const uint32_t intNum;

    /** Interrupt triggering type */
    const ArmInterruptType triggerType;

    /** True if interrupt pin is active, false otherwise */
    bool _active;
};

class ArmSPI : public ArmInterruptPin
{
    friend class ArmSPIGen;

  private:
    ArmSPI(const ArmSPIParams &p);

  public:
    void raise() override;
    void clear() override;
};

class ArmPPI : public ArmInterruptPin
{
    friend class ArmPPIGen;

  private:
    ArmPPI(const ArmPPIParams &p, ThreadContext *tc);

  public:
    void raise() override;
    void clear() override;
};

class ArmSigInterruptPin : public ArmInterruptPin
{
    friend class ArmSigInterruptPinGen;

  private:
    ArmSigInterruptPin(const ArmSigInterruptPinParams &p);

    std::vector<std::unique_ptr<IntSourcePin<ArmSigInterruptPinGen>>> sigPin;

  public:
    void raise() override;
    void clear() override;
};

} // namespace gem5

#endif // __DEV_ARM_BASE_GIC_H__
