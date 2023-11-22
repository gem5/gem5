/*
 * Copyright (c) 2016, 2022-2023 Arm Limited
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

#include "kern/linux/helpers.hh"

#include <regex>
#include <string>
#include <type_traits>
#include <vector>

#include "base/compiler.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "mem/translating_port_proxy.hh"
#include "sim/byteswap.hh"
#include "sim/system.hh"

namespace gem5 {

namespace linux {

namespace {

namespace pre5_10 {

/** Dmesg entry for Linux versions pre-v5.10 */
struct GEM5_PACKED DmesgEntry
{
    uint64_t ts_nsec;
    uint16_t len;
    uint16_t text_len;
    uint16_t dict_len;
    uint8_t facility;
    uint8_t flags;
};

/** Dump a Linux Demsg entry, pre-v5.10. */
static int
dumpDmesgEntry(const uint8_t *base, const uint8_t *end,
               const ByteOrder bo,
               std::ostream &os)
{
    const size_t max_length = end - base;
    DmesgEntry de;

    if (max_length < sizeof(de)) {
        warn("Malformed dmesg entry\n");
        return -1;
    }

    memcpy(&de, base, sizeof(de));
    de.ts_nsec = gtoh(de.ts_nsec, bo);
    de.len = gtoh(de.len, bo);
    de.text_len = gtoh(de.text_len, bo);

    if (de.len < sizeof(de) ||
        max_length < de.len ||
        max_length < sizeof(DmesgEntry) + de.text_len) {

        warn("Malformed dmesg entry:\n");
        warn("\tMax length: %i\n", max_length);
        warn("\tde.len: %i\n", de.len);
        warn("\tde.text_len: %i\n", de.text_len);
        return -1;
    }

    ccprintf(os, "[%.6f] ", de.ts_nsec * 10e-9);
    os.write((char *)base + sizeof(de), de.text_len);
    os << std::endl;

    return de.len;
}

/** Dump the kernel Dmesg ringbuffer for Linux versions pre-v5.10 */
void
dumpDmesg(ThreadContext *tc, std::ostream &os)
{
    System *system = tc->getSystemPtr();
    const ByteOrder bo = system->getGuestByteOrder();
    const auto &symtab = system->workload->symtab(tc);
    TranslatingPortProxy proxy(tc);

    auto lb = symtab.find("__log_buf");
    auto lb_len = symtab.find("log_buf_len");
    auto first = symtab.find("log_first_idx");
    auto next = symtab.find("log_next_idx");

    auto end_it = symtab.end();

    if (lb == end_it || lb_len == end_it ||
            first == end_it || next == end_it) {
        warn("Failed to find kernel dmesg symbols.\n");
        return;
    }

    uint32_t log_buf_len = proxy.read<uint32_t>(lb_len->address(), bo);
    uint32_t log_first_idx = proxy.read<uint32_t>(first->address(), bo);
    uint32_t log_next_idx = proxy.read<uint32_t>(next->address(), bo);

    if (log_first_idx >= log_buf_len || log_next_idx >= log_buf_len) {
        warn("dmesg pointers/length corrupted\n");
        return;
    }

    // Normalize and read the dmesg ring buffer
    std::vector<uint8_t> log_buf(log_buf_len);
    int length;
    if (log_first_idx < log_next_idx) {
        length = log_next_idx - log_first_idx;
        if (length < 0 || length > log_buf.size()) {
            warn("Unexpected dmesg buffer length\n");
            return;
        }
        proxy.readBlob(lb->address() + log_first_idx, log_buf.data(), length);
    } else {
        const int length_2 = log_buf_len - log_first_idx;
        if (length_2 < 0 || length_2 + log_next_idx > log_buf.size()) {
            warn("Unexpected dmesg buffer length\n");
            return;
        }
        length = log_buf_len;
        proxy.readBlob(
            lb->address() + log_first_idx, log_buf.data(), length_2);
        proxy.readBlob(
            lb->address(), log_buf.data() + length_2, log_next_idx);
    }

    // Print dmesg buffer content
    const uint8_t *cur = log_buf.data(), *end = log_buf.data() + length;
    while (cur < end) {
        int ret = dumpDmesgEntry(cur, end, bo, os);
        if (ret < 0)
            return;
        cur += ret;
    }
}

} // namespace pre5_10

namespace post5_10 {

/** Metadata record for the Linux dmesg ringbuffer, post-v5.10.
 *
 *  Struct data members are compatible with the equivalent Linux data
 *  structure. Should be templated on atomic_var_t=int32_t for 32-bit
 *  Linux and atomic_var_t=int64_t for 64-bit Linux.
 *
 *  Also includes helper methods for reading the data structure into
 *  the gem5 world.
 *
 */
template<typename atomic_var_t>
struct GEM5_PACKED DmesgMetadataRecord
{
    using guest_ptr_t = typename std::make_unsigned_t<atomic_var_t>;

    // Struct data members
    atomic_var_t state;
    struct
    {
        guest_ptr_t curr_offset;
        guest_ptr_t next_offset;
    } data_buffer;

    /** Read a DmesgMetadataRecord from guest memory. */
    static DmesgMetadataRecord
    read(const TranslatingPortProxy & proxy,
         Addr address,
         guest_ptr_t data_offset_mask,
         const ByteOrder & bo)
    {
        DmesgMetadataRecord metadata;
        proxy.readBlob(address, &metadata, sizeof(metadata));

        // Convert members to host byte order
        metadata.state = gtoh(metadata.state, bo);
        metadata.data_buffer.curr_offset =
            gtoh(metadata.data_buffer.curr_offset, bo);
        metadata.data_buffer.next_offset =
            gtoh(metadata.data_buffer.next_offset, bo);

        // Mask the offsets
        metadata.data_buffer.curr_offset =
            metadata.data_buffer.curr_offset & data_offset_mask;
        metadata.data_buffer.next_offset =
            metadata.data_buffer.next_offset & data_offset_mask;

        return metadata;
    }
};

/** Info record for the Linux dmesg ringbuffer, post-v5.10.
 *
 *  Struct data members are compatible with the equivalent Linux data
 *  structure. Should be templated on atomic_var_t=int32_t for 32-bit
 *  Linux and atomic_var_t=int64_t for 64-bit Linux.
 *
 *  Also includes helper methods for reading the data structure into
 *  the gem5 world.
 *
 */
struct GEM5_PACKED DmesgInfoRecord
{
    // Struct data members
    uint64_t unused1;
    uint64_t ts_nsec;
    uint16_t message_size;
    uint8_t unused2;
    uint8_t unused3;
    uint32_t unused4;
    struct
    {
        char unused5_1[16];
        char unused5_2[48];
    } unused5;

    /** Read a DmesgInfoRecord from guest memory. */
    static DmesgInfoRecord
    read(const TranslatingPortProxy & proxy,
         Addr address,
         const ByteOrder & bo)
    {
        DmesgInfoRecord info;
        proxy.readBlob(address, &info, sizeof(info));

        // Convert members to host byte order
        info.ts_nsec = gtoh(info.ts_nsec, bo);
        info.message_size = gtoh(info.message_size, bo);

        return info;
    }
};

/** Top-level ringbuffer record for the Linux dmesg ringbuffer, post-v5.10.
 *
 *  Struct data members are compatible with the equivalent Linux data
 *  structure. Should be templated on AtomicVarType=int32_t for 32-bit
 *  Linux and AtomicVarType=int64_t for 64-bit Linux.
 *
 *  Also includes helper methods for reading the data structure into
 *  the gem5 world, and reading/generating appropriate masks.
 *
 */
template<typename AtomicVarType>
struct GEM5_PACKED DmesgRingbuffer
{
    static_assert(
        std::disjunction<
            std::is_same<AtomicVarType, int32_t>,
            std::is_same<AtomicVarType, int64_t>
        >::value,
        "AtomicVarType must be int32_t or int64_t");

    using atomic_var_t = AtomicVarType;
    using guest_ptr_t = typename std::make_unsigned_t<atomic_var_t>;
    using metadata_record_t = DmesgMetadataRecord<atomic_var_t>;

    // Struct data members
    struct
    {
        unsigned int mask_bits;
        guest_ptr_t metadata_ring_ptr;
        guest_ptr_t info_ring_ptr;
        atomic_var_t unused1;
        atomic_var_t unused2;
    } metadata;
    struct
    {
        unsigned int mask_bits;
        guest_ptr_t data_ring_ptr;
        atomic_var_t head_offset;
        atomic_var_t tail_offset;
    } data;
    atomic_var_t fail;

    /** Read a DmesgRingbuffer from guest memory. */
    static DmesgRingbuffer
    read(const TranslatingPortProxy & proxy,
         const Addr address,
         const ByteOrder & bo)
    {
        DmesgRingbuffer rb;
        proxy.readBlob(address, &rb, sizeof(rb));

        // Convert members to host byte order
        rb.metadata.mask_bits =
            gtoh(rb.metadata.mask_bits, bo);
        rb.metadata.metadata_ring_ptr =
            gtoh(rb.metadata.metadata_ring_ptr, bo);
        rb.metadata.info_ring_ptr =
            gtoh(rb.metadata.info_ring_ptr, bo);

        rb.data.mask_bits = gtoh(rb.data.mask_bits, bo);
        rb.data.data_ring_ptr = gtoh(rb.data.data_ring_ptr, bo);
        rb.data.head_offset = gtoh(rb.data.head_offset, bo);
        rb.data.tail_offset = gtoh(rb.data.tail_offset, bo);

        // Mask offsets to the correct number of bits
        rb.data.head_offset =
            rb.mask_data_offset(rb.data.head_offset);
        rb.data.tail_offset =
            rb.mask_data_offset(rb.data.tail_offset);

        return rb;
    }

    /** Make a mask for the bottom mask_bits of an `atomic_var_t`, then
     *  cast it to the required `as_type`.
     */
    template<typename as_type>
    static as_type
    make_offset_mask_as(const unsigned int mask_bits)
    {
        using unsigned_atomic_var_t =
            typename std::make_unsigned<atomic_var_t>::type;
        const atomic_var_t offset_mask =
            static_cast<atomic_var_t>(
                (static_cast<unsigned_atomic_var_t>(1) << mask_bits) - 1);
        return static_cast<as_type>(offset_mask);
    }

    /** Make a mask for an offset into the metadata or info ringbuffers. */
    template<typename metadata_offset_t>
    metadata_offset_t
    make_metadata_offset_mask() const
    {
        return make_offset_mask_as<metadata_offset_t>(metadata.mask_bits);
    }

    /** Make a mask for an offset into the data ringbuffer. */
    template<typename data_offset_t>
    data_offset_t
    make_data_offset_mask() const
    {
        return make_offset_mask_as<data_offset_t>(data.mask_bits);
    }

    /** Apply the correct masking to an offset into the metadata or info
        ringbuffers. */
    template<typename metadata_offset_t>
    metadata_offset_t
    mask_metadata_offset(const metadata_offset_t metadata_offset) const
    {
        const atomic_var_t MASK =
            make_metadata_offset_mask<metadata_offset_t>();
        return metadata_offset & MASK;
    }

    /** Apply the correct masking to an offset into the data ringbuffer. */
    template<typename data_offset_t>
    data_offset_t
    mask_data_offset(const data_offset_t data_offset) const
    {
        const atomic_var_t MASK = make_data_offset_mask<data_offset_t>();
        return data_offset & MASK;
    }
};

// Aliases for the two types of Ringbuffer that could be used.
using Linux64_Ringbuffer = DmesgRingbuffer<int64_t>;
using Linux32_Ringbuffer = DmesgRingbuffer<int32_t>;

/** Print the record at the specified offset into the data ringbuffer,
 *  and return the offset of the next entry in the data ringbuffer,
 *  post-v5.10.
 *
 * The `first_metadata_offset` argument is used to check for
 * wraparound. If the final data record of the ringbuffer is not large
 * enough to hold the message, the record will be left empty and
 * repeated at the beginning of the data ringbuffer. In this case the
 * metadata offset at the beginning of the last record will match the
 * metadata offset of the first record of the ringbuffer, and the last
 * record of the ring buffer should be skipped.
 *
 */
template <typename ringbuffer_t,
          typename atomic_var_t=typename ringbuffer_t::atomic_var_t,
          typename guest_ptr_t=typename ringbuffer_t::guest_ptr_t>
atomic_var_t
iterateDataRingbuffer(std::ostream & os,
                      const TranslatingPortProxy & proxy,
                      const ringbuffer_t & rb,
                      const atomic_var_t offset,
                      const guest_ptr_t first_metadata_offset,
                      const ByteOrder bo)
{
    using metadata_record_t = typename ringbuffer_t::metadata_record_t;

    constexpr size_t METADATA_RECORD_SIZE =
        sizeof(typename ringbuffer_t::metadata_record_t);
    constexpr size_t INFO_RECORD_SIZE = sizeof(DmesgInfoRecord);

    const guest_ptr_t DATA_OFFSET_MASK =
        rb.template make_data_offset_mask<guest_ptr_t>();

    // Read the offset of the metadata record from the beginning of
    // the data record.
    guest_ptr_t metadata_info_offset = rb.mask_metadata_offset(
        proxy.read<guest_ptr_t>(rb.data.data_ring_ptr + offset, bo));

    // If the metadata offset of the block is the same as the metadata
    // offset of the first block of the data ringbuffer, then this
    // data block is unsused (padding), and the iteration can wrap
    // around to the beginning of the data ringbuffer (offset == 0).
    if (metadata_info_offset == first_metadata_offset) {
        return static_cast<atomic_var_t>(0);
    }

    // Read the metadata record from the metadata ringbuffer.
    guest_ptr_t metadata_address =
        rb.metadata.metadata_ring_ptr +
        (metadata_info_offset * METADATA_RECORD_SIZE);
    metadata_record_t metadata =
        metadata_record_t::read(proxy, metadata_address, DATA_OFFSET_MASK, bo);

    // Read the info record from the info ringbuffer.
    guest_ptr_t info_address =
        rb.metadata.info_ring_ptr +
        (metadata_info_offset * INFO_RECORD_SIZE);
    DmesgInfoRecord info =
        DmesgInfoRecord::read(proxy, info_address, bo);

    // The metadata record should point back to the same data record
    // in the data ringbuffer.
    if (metadata.data_buffer.curr_offset != offset) {
        warn_once("Dmesg dump: metadata record (at 0x%08x) does not point "
                  "back to the correponding data record (at 0x%08x). Dmesg "
                  "buffer may be corrupted",
             metadata.data_buffer.next_offset, offset);
    }

    // Read the message from the data record. This is placed
    // immediately after the `guest_ptr_t` sized metadata offset at
    // the beginning of the record.
    std::vector<uint8_t> message(info.message_size);
    proxy.readBlob(rb.data.data_ring_ptr + offset + sizeof(guest_ptr_t),
                   message.data(), info.message_size);

    // Print the record
    ccprintf(os, "[%.6f] ", info.ts_nsec * 10e-9);
    os.write((char *)message.data(), info.message_size);
    os << "\n";

    // Return the offset of the next data record in the data
    // ringbuffer.
    return metadata.data_buffer.next_offset;
}

/** Dump the kernel Dmesg ringbuffer for Linux versions post-v5.10.

    Templated implementation specific to 32-bit or 64-bit Linux.
*/
template <typename ringbuffer_t>
void
dumpDmesgImpl(ThreadContext *tc, std::ostream &os)
{
    using atomic_var_t = typename ringbuffer_t::atomic_var_t;
    using guest_ptr_t = typename ringbuffer_t::guest_ptr_t;

    System *system = tc->getSystemPtr();
    const ByteOrder bo = system->getGuestByteOrder();
    const auto &symtab = system->workload->symtab(tc);
    TranslatingPortProxy proxy(tc);

    auto symtab_end_it = symtab.end();

    // Read the dynamic ringbuffer structure from guest memory, if present.
    ringbuffer_t dynamic_rb;
    auto dynamic_rb_symbol = symtab.find("printk_rb_dynamic");
    if (dynamic_rb_symbol != symtab_end_it) {
        dynamic_rb =
            ringbuffer_t::read(proxy, dynamic_rb_symbol->address(), bo);
    } else {
        warn("Failed to find required dmesg symbols.\n");
        return;
    }

    // Read the static ringbuffer structure from guest memory, if present.
    ringbuffer_t static_rb;
    auto static_rb_symbol = symtab.find("printk_rb_static");
    if (static_rb_symbol != symtab_end_it) {
        static_rb = ringbuffer_t::read(proxy, static_rb_symbol->address(), bo);
    } else {
        warn("Failed to find required dmesg symbols.\n");
        return;
    }

    // Read the pointer to the active ringbuffer structure from guest
    // memory. This should point to one of the two ringbuffer
    // structures already read from guest memory.
    guest_ptr_t active_ringbuffer_ptr = 0x0;
    auto active_ringbuffer_ptr_symbol = symtab.find("prb");
    if (active_ringbuffer_ptr_symbol != symtab_end_it) {
        active_ringbuffer_ptr =
            proxy.read<guest_ptr_t>(active_ringbuffer_ptr_symbol->address(),
                                    bo);
    } else {
        warn("Failed to find required dmesg symbols.\n");
        return;
    }

    if (active_ringbuffer_ptr == 0 ||
        (active_ringbuffer_ptr != dynamic_rb_symbol->address() &&
         active_ringbuffer_ptr != static_rb_symbol->address())) {
        warn("Kernel Dmesg ringbuffer appears to be invalid.\n");
        return;
    }

    ringbuffer_t & rb =
        (active_ringbuffer_ptr == dynamic_rb_symbol->address())
        ? dynamic_rb : static_rb;

    atomic_var_t head_offset = rb.data.head_offset;
    atomic_var_t tail_offset = rb.data.tail_offset;

    // Get some marker offsets into the data ringbuffer which will be
    // used as end values to control the iteration.
    const guest_ptr_t first_metadata_offset = rb.mask_metadata_offset(
        proxy.read<guest_ptr_t>(rb.data.data_ring_ptr, bo));
    const guest_ptr_t invalid_metadata_offset =
        rb.template make_metadata_offset_mask<guest_ptr_t>() + 1;

    // Iterate over the active ringbuffer, printing each message to
    // `os`. Use the maximum number of possible info records plus one
    // (invalid_metadata_offset) as an escape counter to make sure the
    // process doesn't iterate infinitely if the kernel data
    // structures have been corrupted.

    // When head is behind tail, read to the end of the ringbuffer,
    // then loop back to the begining.
    //
    // iterateDataRingbuffer will return offset at the beginning of
    // the data ringbuffer when it loops back.
    //
    // `first_metadata_offset` is used to detect cases where the final data
    // block is unused.
    guest_ptr_t count = 0;
    while (head_offset < tail_offset && count < invalid_metadata_offset) {
        tail_offset =
            iterateDataRingbuffer<ringbuffer_t>(
                os, proxy, rb, tail_offset, first_metadata_offset, bo);
        ++count;
    }

    // When tail is behind head, read forwards from the tail offset to
    // the head offset.
    count = 0;
    while (tail_offset < head_offset && count < invalid_metadata_offset) {
        tail_offset =
            iterateDataRingbuffer<ringbuffer_t>(
                os, proxy, rb, tail_offset, invalid_metadata_offset, bo);
        ++count;
    }
}

/** Extract all null-terminated printable strings from a buffer and
 * return them as a vector.
 *
 */
std::vector<std::string>
extract_printable_strings(const std::vector<uint8_t> buffer)
{
    std::vector<std::string> results;
    std::string result;
    bool reading_printable = false;
    for (const uint8_t byte: buffer) {
        if (std::isprint(byte)) {
            result += static_cast<char>(byte);
            reading_printable = true;
        } else if (reading_printable) {
            if (byte == '\0') {
                results.push_back(result);
            }
            result.clear();
            reading_printable = false;
        }
    }
    return results;
}

/** Try to extract the Linux Kernel Version.
 *
 * This function attempts to find the Kernel version by searching the
 * `uts_namespace` struct at the exported symbol `init_uts_ns`. This
 * structure contains the Kernel version as a string, and is
 * referenced by `procfs` to return the Kernel version in a running
 * system.
 *
 * Different versions of the Kernel use different layouts for this
 * structure, and the top level structure is marked as
 * `__randomize_layout`, so the exact layout in memory cannot be
 * relied on. Because of this, `extract_kernel_version` takes the
 * approach of searching the memory holding the structure for
 * printable strings, and parsing the version from those strings.
 *
 * If a likely match is found, the version is packed into a uint32_t
 * in the standard format used by the Linux kernel (which allows
 * numerical comparison of version numbers) and returned.
 *
 * If no likely match is found, 0x0 is returned to indicate an error.
 *
 */
[[maybe_unused]]
uint32_t
extract_kernel_version(ThreadContext* tc) {
    System *system = tc->getSystemPtr();
    const auto &symtab = system->workload->symtab(tc);
    auto symtab_end_it = symtab.end();

    auto symbol = symtab.find("init_uts_ns");
    if (symbol == symtab_end_it) {
        return 0x0;
    }

    // Use size of `init_uts_ns` in Linux v5.18.0 as a default.
    // (e.g. for upgraded checkpoints.)
    const size_t INIT_UTS_NS_SIZE_DEFAULT = 432;
    const size_t BUFFER_SIZE =
        symbol->sizeOrDefault(INIT_UTS_NS_SIZE_DEFAULT);

    TranslatingPortProxy proxy(tc);
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    proxy.readBlob(
        symbol->address(), buffer.data(), buffer.size() * sizeof(uint8_t));
    auto strings = extract_printable_strings(buffer);

    const std::regex version_re {"^(\\d+)\\.(\\d+)\\.(\\d)+$"};
    std::smatch match;
    for (const auto& string: strings) {
        if (std::regex_search(string, match, version_re)) {
            try {
                int major = std::stoi(match[1]);
                int minor = std::stoi(match[2]);
                int point = std::stoi(match[3]);
                return (
                    (major & 0xFF) << 16
                    | (minor & 0xFF) << 8
                    | std::min(point, 255));
            }
            catch (const std::invalid_argument &) {
                // This shouldn't be possible if the regex matched.
                continue;
            }
            catch (const std::out_of_range &) {
                continue;
            }
        }
    }

    return 0x0;
}

/** Dump the kernel Dmesg ringbuffer for Linux versions post-v5.10.
 *
 *  Delegates to an architecture specific template funtion instance.
 *
 */
void
dumpDmesg(ThreadContext *tc, std::ostream &os)
{
    System *system = tc->getSystemPtr();
    const bool os_is_64_bit = loader::archIs64Bit(system->workload->getArch());

    if (os_is_64_bit) {
        dumpDmesgImpl<Linux64_Ringbuffer>(tc, os);
    } else {
        dumpDmesgImpl<Linux32_Ringbuffer>(tc, os);
    }
}

} // namespace post5_10

} // anonymous namespace

void
dumpDmesg(ThreadContext *tc, std::ostream &os)
{
    System *system = tc->getSystemPtr();
    const auto &symtab = system->workload->symtab(tc);

    auto end_it = symtab.end();

    // Search for symbols associated with the Kernel Dmesg ringbuffer,
    // pre-v5.10.
    auto lb = symtab.find("__log_buf");
    auto lb_len = symtab.find("log_buf_len");
    auto first = symtab.find("log_first_idx");
    auto next = symtab.find("log_next_idx");

    if (lb != end_it && lb_len != end_it &&
            first != end_it && next != end_it) {
        linux::pre5_10::dumpDmesg(tc, os);
        return;
    }

    // Search for symbols associated with the Kernel Dmesg ringbuffer,
    // post-v5.10.
    auto printk_rb_static = symtab.find("printk_rb_static");
    auto printk_rb_dynamic = symtab.find("printk_rb_dynamic");

    if (printk_rb_dynamic != end_it || printk_rb_static != end_it) {
        linux::post5_10::dumpDmesg(tc, os);
        return;
    }

    // Required symbols relating to the Kernel Dmesg buffer were not
    // found for any supported version of Linux.
    warn("Failed to find kernel dmesg symbols.\n");
}

} // namespace linux

} // namespace gem5
