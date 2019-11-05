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

class Base64Delta16(BaseDictionaryCompressor):
    type = 'Base64Delta16'
    cxx_class = 'Compressor::Base64Delta16'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 64

class Base64Delta32(BaseDictionaryCompressor):
    type = 'Base64Delta32'
    cxx_class = 'Compressor::Base64Delta32'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 64

class Base32Delta8(BaseDictionaryCompressor):
    type = 'Base32Delta8'
    cxx_class = 'Compressor::Base32Delta8'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 32

class Base32Delta16(BaseDictionaryCompressor):
    type = 'Base32Delta16'
    cxx_class = 'Compressor::Base32Delta16'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 32

class Base16Delta8(BaseDictionaryCompressor):
    type = 'Base16Delta8'
    cxx_class = 'Compressor::Base16Delta8'
    cxx_header = "mem/cache/compressors/base_delta.hh"

    chunk_size_bits = 16

class CPack(BaseDictionaryCompressor):
    type = 'CPack'
    cxx_class = 'Compressor::CPack'
    cxx_header = "mem/cache/compressors/cpack.hh"

class FPCD(BaseDictionaryCompressor):
    type = 'FPCD'
    cxx_class = 'Compressor::FPCD'
    cxx_header = "mem/cache/compressors/fpcd.hh"

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
    extra_decomp_lat = Param.Unsigned(0, "Extra latency to be added to the "
        "sub-compressor's decompression latency")

class PerfectCompressor(BaseCacheCompressor):
    type = 'PerfectCompressor'
    cxx_class = 'Compressor::Perfect'
    cxx_header = "mem/cache/compressors/perfect.hh"

    chunk_size_bits = 64
    max_compression_ratio = Param.Int(Parent.max_compression_ratio,
        "Maximum compression ratio allowed")
    compression_latency = Param.Cycles(1,
        "Number of cycles to perform data compression")
    decompression_latency = Param.Cycles(1,
        "Number of cycles to perform data decompression")

class RepeatedQwordsCompressor(BaseDictionaryCompressor):
    type = 'RepeatedQwordsCompressor'
    cxx_class = 'Compressor::RepeatedQwords'
    cxx_header = "mem/cache/compressors/repeated_qwords.hh"

    chunk_size_bits = 64

class ZeroCompressor(BaseDictionaryCompressor):
    type = 'ZeroCompressor'
    cxx_class = 'Compressor::Zero'
    cxx_header = "mem/cache/compressors/zero.hh"

    chunk_size_bits = 64

class BDI(MultiCompressor):
    encoding_in_tags=True
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
