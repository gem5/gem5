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

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <new>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include <string>
#include <vector>

#include <ARM/TLM/arm_chi_payload.h>

#ifdef ARM_TLM_ENABLE_VALGRIND
#include <valgrind/memcheck.h>
#else
#define VALGRIND_CREATE_MEMPOOL(d1, d2, d3) do {} while (0)
#define VALGRIND_DESTROY_MEMPOOL(d1) do {} while (0)
#define VALGRIND_MEMPOOL_ALLOC(d1, d2, d3) do {} while (0)
#define VALGRIND_MEMPOOL_FREE(d1, d2) do {} while (0)
#endif

#ifndef ARM_TLM_EXPORT
#define ARM_TLM_EXPORT
#endif

#ifdef ARM_TLM_ERRORS_WITH_ASSERT
/* Force use of assert() for error reporting. */
#define runtime_error_assert(cond) do { assert(cond); } while (0)
#else
/*
 * Throw a std::runtime_exception for assertions as it may be possible
 * to catch/handle that that a hard assertion.
 */
#define runtime_error_assert(cond) do { \
    if (!(cond)) \
    { \
        std::ostringstream message; \
        message << "Assertion at: " __FILE__ ":" << __LINE__ << ": " #cond; \
        throw std::runtime_error(message.str()); \
    } \
} while (0)
#endif

namespace ARM
{
namespace CHI
{

ARM_TLM_EXPORT ARM_CHI_TLM_API_VERSION::ARM_CHI_TLM_API_VERSION(){}

/**
 * Source of all allocated Payloads. The PayloadPool manages the memory of
 * Payloads and issuing unique IDs to Payloads. PayloadPool never deallocates
 * any memory for Payload object but maintains a free list of objects returned
 * to the pool from which new objects are created.
 */
class PayloadPool
{
private:
    /** The unique ID of the next Payload to be created. */
    uint64_t next_uid;

    /**
     * Size of the Payload object will all registered extensions. All Payloads
     * will have the same size.
     */
    unsigned payload_size;

    /**
     * Set to true once the first Payload is requested. Extensions may not be
     * registered once pool_fixed becomes true.
     */
    bool pool_fixed;

    /** Number of allocated Payloads in existence. */
    std::size_t allocated_payload_count;

    /** LIFO free list of allocated Payload objects. */
    std::vector<Payload*> payload_pool;

    /** ExtensionEntry class holidng an extensuion manager and offset */
    class ExtensionEntry
    {
    public:
        std::size_t offset;
        PayloadExtensionManager* manager;
        ExtensionEntry(std::size_t offset_, PayloadExtensionManager* manager_):
            offset(offset_),
            manager(manager_)
        {}
        ExtensionEntry(){}
    };
    /** Map of extension names to ExtensionEntries */
    std::map<std::string, ExtensionEntry> extension_map;

    /** Debug allocation of new payloads. */
    bool debug_unique;
    bool debug_always_free;

public:
    /** Dummy payload for credit passing etc.*/
    Payload dummy_payload;

    /** Registered new/delete for allocations within the pool. */
    void* (*local_malloc)(size_t size);
    void (*local_free)(void* ptr);

    /** Return the next unique ID and advance next_id. */
    uint64_t get_uid() { return next_uid++; }

    /** Return a Payload to the payload free list. */
    void free_payload(Payload* payload)
    {
        for (std::map<std::string, ExtensionEntry>::iterator it =
            extension_map.begin(); it!=extension_map.end(); ++it)
        {
            ExtensionEntry ext = it->second;
            ext.manager->destroy(
                reinterpret_cast<char*>(payload) + ext.offset);
        }
        VALGRIND_MEMPOOL_FREE(payload, payload);
        VALGRIND_DESTROY_MEMPOOL(payload);
        if (debug_always_free)
        {
            local_free(payload);
            allocated_payload_count--;
        } else if (!debug_unique)
        {
            payload_pool.push_back(payload);
        }
    }

    PayloadPool() :
        next_uid(1),
        /* payload_size wil grow as extensions are added. */
        payload_size(sizeof(Payload)),
        pool_fixed(false),
        allocated_payload_count(0),
        debug_unique(false),
        debug_always_free(false),
        dummy_payload(0),
        local_malloc(std::malloc),
        local_free(std::free)
    {
        char* env_val = getenv("ARM_TLM_DEBUG_ALLOC");
        if (env_val)
        {
            debug_unique = (std::string(env_val) == "UNIQUE");
            debug_always_free = (std::string(env_val) == "ALWAYS_FREE");
        }
    }

    /**
     * Create a new Payload either by allocating a new object or recycling a
     * Payload from the payload free list.
     */
    Payload* new_payload(Payload* parent = nullptr)
    {
        Payload* payload;

        if (payload_pool.empty())
        {
            pool_fixed = true;
            payload = reinterpret_cast<Payload*>(local_malloc(payload_size));
            allocated_payload_count++;
        } else
        {
            payload = payload_pool.back();
            payload_pool.pop_back();
        }

        VALGRIND_CREATE_MEMPOOL(payload, 0, 0);
        VALGRIND_MEMPOOL_ALLOC(payload, payload, payload_size);

        /* Copy the parent or clear all the payload's data members. */
        if (parent)
        {
            for (std::map<std::string, ExtensionEntry>::iterator it =
                extension_map.begin(); it!=extension_map.end(); ++it)
            {
                ExtensionEntry &ext = it->second;

                ext.manager->copy(
                    reinterpret_cast<char*>(payload) + ext.offset,
                    reinterpret_cast<const char*>(parent) + ext.offset);
            }
        }
        else
        {
            for (std::map<std::string, ExtensionEntry>::iterator it =
                extension_map.begin(); it!=extension_map.end(); ++it)
            {
                ExtensionEntry ext = it->second;
                ext.manager->create(
                    reinterpret_cast<char*>(payload) + ext.offset);
            }
        }

        /*
         * The created payload must still be placement constructed to
         * initialize its required data members.
         */
        return payload;
    }

    void copy_response(Payload* dst, Payload* src)
    {
        for (std::map<std::string, ExtensionEntry>::iterator it =
            extension_map.begin(); it!=extension_map.end(); ++it)
        {
            ExtensionEntry ext = it->second;
            ext.manager->copy_response(
                reinterpret_cast<char*>(dst) + ext.offset,
                reinterpret_cast<const char*>(src) + ext.offset);
        }
    }

    /**
     * Register or find an extension's byte offset within Payload objects. The
     * first call with any particular name will grow payload_size and register
     * the offset of required extension with extension_map.
     *
     * This function can only be called before the PayloadPool has been fixed at
     * its first Payload creation event.
     */

    std::size_t get_extension_offset(const char* name)
    {
        std::map<std::string, ExtensionEntry>::iterator it =
            extension_map.find(name);
        if (it == extension_map.end())
            return 0;
        return it->second.offset;
    }

    std::size_t register_extension(const char* name, PayloadExtensionManager* manager)
    {
        runtime_error_assert(pool_fixed == false);
        /* 32 bit align the payload_size. */
        std::size_t extension_offset = (payload_size + 3) & ~3;

        extension_map[name] = ExtensionEntry(extension_offset, manager);
        payload_size = static_cast<unsigned>(extension_offset + manager->get_size());

        return extension_offset;
    }

    /** Dump debugging info. */
    void debug(std::ostream& stream);
};

/**
 * Global variable. Must not be replicated between code/libraries in the same
 * program.
 */
PayloadPool* global_payload_pool = nullptr;

/** Make the global pool on request. */
PayloadPool* get_global_pool()
{
    if (global_payload_pool == nullptr)
        global_payload_pool = new PayloadPool();
    return global_payload_pool;
}

Payload::Payload(uint64_t uid_) :
    refcount(1),
    uid(uid_),
    parent(nullptr),
    address(0),
    size(0),
    mem_attr(0),
    lpid(0),
    exclusive(0),
    snoop_me(0),
    endian(0),
    stash_nid_valid(0),
    deep(0),
    ns(0),
    likely_shared(0),
    ret_to_src(0),
    do_not_go_to_sd(0),
    do_not_data_pull(0),
    rsvdc(0),
    data_pull(0),
    data_source(0),
    mpam(),
    tu(0),
    byte_enable(0)
{
    std::fill(tag, tag + 4, 0);
    std::fill(data, data + 64, 0);

}

Payload::Payload(Payload* parent_, uint64_t uid_) :
    refcount(1),
    uid(uid_),
    parent(parent_),
    address(parent_->address),
    size(parent_->size),
    mem_attr(parent_->mem_attr),
    lpid(parent_->lpid),
    exclusive(parent_->exclusive),
    snoop_me(parent_->snoop_me),
    endian(parent_->endian),
    stash_nid_valid(parent_->stash_nid_valid),
    deep(parent_->deep),
    ns(parent_->ns),
    likely_shared(parent_->likely_shared),
    ret_to_src(parent_->ret_to_src),
    do_not_go_to_sd(parent_->do_not_go_to_sd),
    do_not_data_pull(parent_->do_not_data_pull),
    rsvdc(parent_->rsvdc),
    data_pull(parent_->data_pull),
    data_source(parent_->data_source),
    mpam(parent_->mpam),
    tu(parent_->tu),
    byte_enable(parent_->byte_enable)
{
    std::copy(parent->tag, parent->tag + 4, tag);
    std::copy(parent->data, parent->data + 64, data);
    parent->ref();
}

Payload::~Payload()
{
    if (parent)
        parent->unref();
}

Payload::Payload(const Payload&) :
    uid(get_global_pool()->get_uid()),
    parent(nullptr)
{
    /*
     * This implementation should never be called but sets the const members
     * of Payload to avoid compilation errors.
     */
}

ARM_TLM_EXPORT Payload* Payload::new_payload()
{
    PayloadPool* pool = get_global_pool();
    Payload* payload = pool->new_payload();
    new (payload) Payload(pool->get_uid());

    return payload;
}

ARM_TLM_EXPORT void Payload::operator delete (void* p)
{
    get_global_pool()->free_payload(reinterpret_cast<Payload*>(p));
}

ARM_TLM_EXPORT Payload* Payload::descend()
{
    PayloadPool* pool = get_global_pool();
    Payload* payload = pool->new_payload(this);
    new (payload) Payload(this, pool->get_uid());

    return payload;
}

ARM_TLM_EXPORT void Payload::propagate_response(Payload* target)
{
    if (!target)
        target = parent;
    target->data_pull = data_pull;
    target->data_source = data_source;

    PayloadPool* pool = get_global_pool();
    pool->copy_response(target, this);
}

ARM_TLM_EXPORT void Payload::ref() const
{
    /* A refcount that's already 0 implies that this payload has previously been deleted and not yet reallocated. */
    runtime_error_assert(refcount > 0);
    refcount++;
}

ARM_TLM_EXPORT void Payload::unref() const
{
    runtime_error_assert(refcount);
    refcount--;
    if (refcount == 0)
        delete this;
}

ARM_TLM_EXPORT Payload* Payload::get_dummy()
{
    return &get_global_pool()->dummy_payload;
}

ARM_TLM_EXPORT std::size_t Payload::get_extension_offset(const char* name)
{
    return get_global_pool()->get_extension_offset(name);
}

ARM_TLM_EXPORT std::size_t Payload::register_extension(const char* name,
    PayloadExtensionManager* manager)
{
    return get_global_pool()->register_extension(name, manager);
}

void PayloadPool::debug(std::ostream& stream)
{
    stream << "Payloads free/allocated/in use:     "
        << payload_pool.size()
        << '/' << allocated_payload_count
        << '/' << allocated_payload_count - payload_pool.size() << '\n';
}

ARM_TLM_EXPORT void Payload::debug_payload_pool(std::ostream& stream)
{
    get_global_pool()->debug(stream);
}

}
}
