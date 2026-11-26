/*
 * The Clear BSD License
 *
 * Copyright (c) 2015-2021 Arm Limited.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ARM_CHI_PAYLOAD_H
#define ARM_CHI_PAYLOAD_H

#include <stdint.h>
#include <cstddef>
#include <iostream>

#include <ARM/TLM/arm_tlm_helpers.h>

namespace ARM
{
namespace CHI
{

#define ARM_CHI_TLM_API_VERSION ARM_CHI_TLM_API_1
struct ARM_CHI_TLM_API_VERSION{ARM_CHI_TLM_API_VERSION();};
static ARM_CHI_TLM_API_VERSION api_version_check;

class PayloadExtensionManager;

/** CHI SIZE field. */
enum SizeEnum
{
    SIZE_1   = 0,
    SIZE_2   = 1,
    SIZE_4   = 2,
    SIZE_8   = 3,
    SIZE_16  = 4,
    SIZE_32  = 5,
    SIZE_64  = 6
};

/**
 * CHI MemAttr. This is constructed entirely from MemAttrBitEnum. NB: Needs to be combined with Order for the full
 * effect.
 */
enum MemAttrEnum
{
    MEM_ATTR_DEVICE_NE      = 0x2, ///< With ORDER_ENDPOINT_ORDER: Device nRnE, invalid otherwise.
    MEM_ATTR_DEVICE_E       = 0x3, ///< With ORDER_ENDPOINT_ORDER: Device nRE, otherwise Device RE
    MEM_ATTR_NORMAL_NC_NB   = 0x0, ///< Normal Non-cacheable Non-bufferable (use with ORDER_NO_ORDER or
                                   ///< ORDER_REQUEST_ORDER)
    MEM_ATTR_NORMAL_NC_B    = 0x1, ///< Normal Non-cacheable bufferable (use with ORDER_NO_ORDER or ORDER_REQUEST_ORDER)
    MEM_ATTR_NORMAL_WB_NA   = 0x5, ///< Normal Write-back No-allocate (use with ORDER_NO_ORDER or ORDER_REQUEST_ORDER)
    MEM_ATTR_NORMAL_WB_A    = 0xd, ///< Normal Write-back Allocate (use with ORDER_NO_ORDER or ORDER_REQUEST_ORDER)

    // No other valid values as of AMBA 5 CHI rev C
};

enum MemAttrBitEnum
{
    MEM_ATTR_EWA             = 1,
    MEM_ATTR_DEVICE          = 2,
    MEM_ATTR_CACHEABLE       = 4,
    MEM_ATTR_ALLOCATE        = 8
};

/** CHI DataPull field. */
enum DataPullEnum
{
    DATA_PULL_NO_READ = 0,
    DATA_PULL_READ    = 1
};

/**
 * Typedefs wrapping enumeration types for Payload data members in EnumWrapper
 * and BitEnumWrapper.
 */
typedef TLM::EnumWrapper<SizeEnum, uint8_t> Size;
typedef TLM::BitEnumWrapper<MemAttrEnum, MemAttrBitEnum, uint8_t> MemAttr;
typedef TLM::EnumWrapper<DataPullEnum, uint8_t> DataPull;

class Mpam
{
public:
    uint8_t mpam_ns;
    uint16_t part_id;
    uint8_t perf_mon_group;

    Mpam() :
        mpam_ns(0),
        part_id(0),
        perf_mon_group(0)
    {}

    Mpam(uint8_t mpam_ns_, uint16_t part_id_, uint8_t perf_mon_group_) :
        mpam_ns(mpam_ns_),
        part_id(part_id_),
        perf_mon_group(perf_mon_group_)
    {}
};

/** CHI transaction payload. */
class Payload
{
    friend class PayloadPool;

private:
    /**
     * Payload reference count. New payloads have a reference count of 1. The
     * reference count can be incremented/decremented with ref/unref. When
     * unref is called on a payload with refcount == 1, the payload will be
     * returned to the payload pool.
     */
    mutable unsigned refcount;

public:
    /**
     * Unique ID of this payload. Unique IDs are set when a payload is created
     * (using new_payload, clone, or descend) and are guarenteed to be unique
     * within a simulation regardless of reuse of a payload's memory.
     */
    uint64_t const uid;

    /**
     * 'Parent' payload from which this payload is descended or cloned. The
     * parent/child relationship can be used to chain payloads which form part
     * of the same logical transaction but which must exist as separate Payload
     * objects
     */
    Payload* const parent;

public:
    uint64_t address;
    Size size;
    MemAttr mem_attr;
    union
    {
        uint8_t lpid;
        uint8_t p_group_id;
        uint8_t stash_group_id;
        uint8_t tag_group_id;
    };

    bool exclusive         : 1;
    bool snoop_me          : 1;
    bool endian            : 1;
    bool stash_nid_valid   : 1;
    bool deep              : 1;
    bool ns                : 1;
    bool likely_shared     : 1;
    bool ret_to_src        : 1;
    bool do_not_go_to_sd   : 1;
    bool do_not_data_pull  : 1;

    uint32_t rsvdc;
    DataPull data_pull;
    uint8_t data_source;
    Mpam mpam;

    uint8_t tu;
    uint8_t tag[4];

    uint8_t data[64];
    uint64_t byte_enable;

private:
    /** Create a payload, Called by new_payload. */
    explicit Payload(uint64_t uid_);

    /** Create a payload with a parent. Called by descend. */
    explicit Payload(Payload* parent_, uint64_t uid_);

    /** Forbid copy construction. */
    Payload(const Payload&);

    /** Forbid assignment. */
    Payload& operator= (const Payload&) { return *this; }

    /** Forbid public delete as Payloads are reference counted. */
    void operator delete (void* p);

    /** Forbid public stack-based objects. */
    ~Payload();

public:
    /** Increment reference count. */
    void ref() const;

    /** Decrement reference count. */
    void unref() const;

    /**
     * Create a new payload without a parent. All data members are reset to 0.
     */
    static Payload* new_payload();

    /**
     * Create a new payload setting the parent to 'this'. All data members are
     * copied from the parent.
     */
    Payload* descend();

    /**
     * Copy the response fields to the parent payload
     */
    void propagate_response(Payload* target = nullptr);

    /**
     * Get a dummy payload for non payload TLM calls (credit passing etc).
     */
    static Payload* get_dummy();

    /**
     * Get the offset from the start of a Payload of the named extension.
     * Returns 0 if the extension has not been registered.
     * This function will typically only be called by PayloadExtension<>'s
     * constructor.
     */
    static std::size_t get_extension_offset(const char* name);

    /**
     * Register a new named extension
     * The manager provides the extension type's construct, copy and destruct
     * functionality as well as the size.
     * This function will typically only be called by PayloadExtension<>'s
     * constructor.
     */
    static std::size_t register_extension(const char* name,
        PayloadExtensionManager* manager);

    /**
     * Print debugging information for the payload pool. Useful for debugging
     * memory problems.
     */
    static void debug_payload_pool(std::ostream& stream);
};

/**
 * Base class of a PayloadExtensionManageer. This provides the functionality
 * of creating, copying and destroying extensions on payloads. The get_size
 * function should return the size in memory the extension occupies.
 */
class PayloadExtensionManager
{
public:
    virtual void create(void* /* ext */) {}
    virtual void destroy(void* /* ext */) {}
    virtual void copy(void* /* dst */, const void* /* src */) {}
    virtual void copy_response(void* /* dst */, const void* /* src */) {}
    virtual std::size_t get_size() = 0;

};

/**
 * A default PayloadExtensionManager which calls the extension type's
 * constructor, copy constructor and destructor. If an extension manager is not
 * specified, this will be used.
 */
template <typename Type>
class PayloadExtensionManagerTyped:
    public PayloadExtensionManager
{
public:
    void create(void* vext)
    {
        Type* ext = reinterpret_cast<Type*>(vext);
        new (ext) Type();
    }

    void destroy(void* vext)
    {
        Type* ext = reinterpret_cast<Type*>(vext);
        ext->~Type();
    }

    void copy(void* vdst, const void* vsrc)
    {
        const Type* src = reinterpret_cast<const Type*>(vsrc);
        Type* dst = reinterpret_cast<Type*>(vdst);
        new(dst) Type(*src);
    }

    std::size_t get_size()
    {
        return sizeof(Type);
    }
};

/**
 * A default ResponsePayloadExtensionManager which calls the extension type's
 * constructor, copy constructor and destructor. It also executes copy_response
 * to the parent.
 */
template <typename Type>
class ResponsePayloadExtensionManagerTyped:
    public PayloadExtensionManagerTyped <Type>
{
public:
    void copy_response(void* vdst, const void* vsrc)
    {
        const Type* src = reinterpret_cast<const Type*>(vsrc);
        Type* dst = reinterpret_cast<Type*>(vdst);
        new(dst) Type(*src);
    }
};

/**
 * Extensions to CHI::Payload. Extensions must be registered by (creating a
 * PayloadExtension object on the extension's type, or calling
 * get_extension_offset) before any CHI::Payloads are made in a system.
 */
template <typename Type, typename ManagerType = PayloadExtensionManagerTyped<Type> >
class PayloadExtension
{
private:
    /** The offset into all Payloads where the extension can be found. */
    std::size_t offset;

public:
    /**
     * Constructor which registers an extension with the extension map managed
     * by the payload pool.
     */
    explicit PayloadExtension(const char* name)
    {
        offset = Payload::get_extension_offset(name);
        if (offset == 0)
            offset = Payload::register_extension(name, new ManagerType());
    }

    /** Get an extension from a Payload object. */
    Type& get(Payload* payload)
    {
        return *reinterpret_cast<Type*>(
            reinterpret_cast<char*>(payload) + offset);
    }

    /** Set an extension (using operator=). */
    void set(Payload* payload, const Type& value)
    {
        get(payload) = value;
    }
};

/**
 * Response extension which is copied to the parent payload on propagate_response calls
 */
template <typename Type>
class ResponsePayloadExtension :
    public PayloadExtension <Type, ResponsePayloadExtensionManagerTyped<Type>>
{
public:
    explicit ResponsePayloadExtension(const char* name):
        PayloadExtension<Type, ResponsePayloadExtensionManagerTyped<Type>>(name)
    {}

};

}
}

#endif /* ARM_CHI_PAYLOAD_H */
