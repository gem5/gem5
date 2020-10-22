# Copyright (c) 2018-2020 Inria
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

class BaseCacheCompressor(SimObject):
    type = 'BaseCacheCompressor'
    abstract = True
    cxx_class = 'Compressor::Base'
    cxx_header = "mem/cache/compressors/base.hh"

    block_size = Param.Int(Parent.cache_line_size, "Block size in bytes")
    chunk_size_bits = Param.Unsigned(32,
        "Size of a parsing data chunk (in bits)")
    size_threshold_percentage = Param.Percent(50,
        "Minimum percentage of the block size, a compressed block must "
        "achieve to be stored in compressed format")

    comp_chunks_per_cycle = Param.Unsigned(1,
        "Number of chunks that can be compressed in parallel per cycle.")
    comp_extra_latency = Param.Cycles(1, "Number of extra cycles required "
        "to finish compression (e.g., due to shifting and packaging).")
    decomp_chunks_per_cycle = Param.Unsigned(1,
        "Number of chunks that can be decompressed in parallel per cycle.")
    decomp_extra_latency = Param.Cycles(1, "Number of extra cycles required "
        "to finish decompression (e.g., due to shifting and packaging).")

class BaseDictionaryCompressor(BaseCacheCompressor):
    type = 'BaseDictionaryCompressor'
    abstract = True
    cxx_class = 'Compressor::BaseDictionaryCompressor'
    cxx_header = "mem/cache/compressors/dictionary_compressor.hh"

    dictionary_size = Param.Int(Parent.cache_line_size,
        "Number of dictionary entries")

class Base64Delta8(BaseDictionaryCompressor):
    type = 'Base64Delta8'
    cxx_class = 'Compressor::Base64Delta8'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 64

    # Base-delta compressors achieve 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class Base64Delta16(BaseDictionaryCompressor):
    type = 'Base64Delta16'
    cxx_class = 'Compressor::Base64Delta16'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 64

    # Base-delta compressors achieve 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class Base64Delta32(BaseDictionaryCompressor):
    type = 'Base64Delta32'
    cxx_class = 'Compressor::Base64Delta32'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 64

    # Base-delta compressors achieve 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class Base32Delta8(BaseDictionaryCompressor):
    type = 'Base32Delta8'
    cxx_class = 'Compressor::Base32Delta8'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 32

    # Base-delta compressors achieve 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class Base32Delta16(BaseDictionaryCompressor):
    type = 'Base32Delta16'
    cxx_class = 'Compressor::Base32Delta16'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 32

    # Base-delta compressors achieve 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class Base16Delta8(BaseDictionaryCompressor):
    type = 'Base16Delta8'
    cxx_class = 'Compressor::Base16Delta8'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 16

    # Base-delta compressors achieve 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class CPack(BaseDictionaryCompressor):
    type = 'CPack'
    cxx_class = 'Compressor::CPack'
    cxx_header = "mem/cache/compressors/cpack.hh"

    comp_chunks_per_cycle = 2
    # Accounts for pattern matching, length generation, packaging and shifting
    comp_extra_latency = 5
    decomp_chunks_per_cycle = 2
    decomp_extra_latency = 1

class FPCD(BaseDictionaryCompressor):
    type = 'FPCD'
    cxx_class = 'Compressor::FPCD'
    cxx_header = "mem/cache/compressors/fpcd.hh"

    # Accounts for checking all patterns, selecting patterns, and shifting
    # The original claim of a decompression latency of 2 cycles would likely
    # generate an unrealistically complex circuit
    comp_chunks_per_cycle = 4
    comp_extra_latency = 1
    decomp_chunks_per_cycle = 4
    decomp_extra_latency = 0

    dictionary_size = 2

class MultiCompressor(BaseCacheCompressor):
    type = 'MultiCompressor'
    cxx_class = 'Compressor::Multi'
    cxx_header = "mem/cache/compressors/multi.hh"

    # Dummy default compressor list. This might not be an optimal choice,
    # since these compressors have many overlapping patterns
    compressors = VectorParam.BaseCacheCompressor([CPack(), FPCD()],
        "Array of compressors")
    encoding_in_tags = Param.Bool(False, "If set the bits to inform which "
        "sub-compressor compressed some data are added to its corresponding "
        "tag entry.")

    # Use the sub-compressors' latencies
    comp_chunks_per_cycle = 0
    decomp_chunks_per_cycle = 0

    # Assume extra 1 cycle to select the results of the winning sub-compressor
    comp_extra_latency = 1

    # Multi-compressors may need a couple of extra cycles to the select
    # which sub-compressor should be used to decompress the data
    decomp_extra_latency = 1

class PerfectCompressor(BaseCacheCompressor):
    type = 'PerfectCompressor'
    cxx_class = 'Compressor::Perfect'
    cxx_header = "mem/cache/compressors/perfect.hh"

    chunk_size_bits = 64

    max_compression_ratio = Param.Int("Maximum compression ratio allowed")

    # In a perfect world compression and decompression happen in 1 cycle
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class RepeatedQwordsCompressor(BaseDictionaryCompressor):
    type = 'RepeatedQwordsCompressor'
    cxx_class = 'Compressor::RepeatedQwords'
    cxx_header = "mem/cache/compressors/repeated_qwords.hh"

    chunk_size_bits = 64

    # Assume 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class ZeroCompressor(BaseDictionaryCompressor):
    type = 'ZeroCompressor'
    cxx_class = 'Compressor::Zero'
    cxx_header = "mem/cache/compressors/zero.hh"

    chunk_size_bits = 64

    # Assume 1-cycle latencies
    comp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    comp_extra_latency = 0
    decomp_chunks_per_cycle = 8 * Self.block_size / Self.chunk_size_bits
    decomp_extra_latency = 0

class BDI(MultiCompressor):
    compressors = [
        ZeroCompressor(size_threshold_percentage=99),
        RepeatedQwordsCompressor(size_threshold_percentage=99),
        Base64Delta8(size_threshold_percentage=99),
        Base64Delta16(size_threshold_percentage=99),
        Base64Delta32(size_threshold_percentage=99),
        Base32Delta8(size_threshold_percentage=99),
        Base32Delta16(size_threshold_percentage=99),
        Base16Delta8(size_threshold_percentage=99),
    ]

    # By default assume that the encoding is stored in the tags, and is
    # retrieved and decoded while (and ends before) the data is being read.
    decomp_extra_latency = 0
    encoding_in_tags=True
