# Copyright (c) 2026, University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Below are pre-defined cache activation energy constants
# based on the McPAT Power Model (which, is based on CACTI-7.0 by extension)
# Note that these are exclusively for a Private L1I/D$
# and Shared L2$ of 32kB/32kB/1MB, 8/4/8 associativities
# respectively.

# While you can technically use these on caches of other sizes/assocs.
# the obtained power values will be misrepresentitive of what McPAT
# will actually output and you will lose accuracy.

cache_act_energies = {
    "DataCacheData": 7.80998e-12,
    "DataCacheTag": 1.57793e-12,
    "DataCache": {"Write": 1.57793e-12},
    "DataCacheMissb": {
        "Read": 6.13188e-12,
        "Write": 6.03686e-12,
        "Search": 5.31524e-12,
    },
    "DataCacheIfb": {
        "Read": 3.19263e-12,
        "Write": 3.17392e-12,
        "Search": 2.93613e-12,
    },
    "DataCachePrefetchb": {
        "Read": 3.19263e-12,
        "Write": 3.17392e-12,
        "Search": 2.93613e-12,
    },
    "DataCacheWritebackb": {
        "Read": 3.19263e-12,
        "Write": 3.17392e-12,
        "Search": 2.93613e-12,
    },
    "InstCacheData": 7.80998e-12,
    "InstCacheTag": 1.57793e-12,
    "InstCache": {"Write": 1.77257e-11},
    "InstCacheMissb": {
        "Read": 6.13188e-12,
        "Write": 6.03686e-12,
        "Search": 5.31524e-12,
    },
    "InstCacheIfb": {
        "Read": 3.19263e-12,
        "Write": 3.17392e-12,
        "Search": 2.93613e-12,
    },
    "InstCachePrefetchb": {
        "Read": 3.19263e-12,
        "Write": 3.17392e-12,
        "Search": 2.93613e-12,
    },
    "InstCacheWritebackb": {
        "Read": 3.19263e-12,
        "Write": 3.17392e-12,
        "Search": 2.93613e-12,
    },
    "L2Cache": {"Write": 1.80156e-10},
    "L2CacheData": 1.52711e-10,
    "L2CacheTag": {"Read": 7.0062e-12, "Write": 2.10746e-11},
    "L2CacheMissb": {
        "Read": 7.44104e-12,
        "Write": 7.67662e-12,
        "Search": 7.79179e-12,
    },
    "L2CacheIfb": {
        "Read": 3.72729e-11,
        "Write": 3.7854e-11,
        "Search": 3.5083e-11,
    },
    "L2CachePrefetchb": {
        "Read": 3.72729e-11,
        "Write": 3.7854e-11,
        "Search": 3.5083e-11,
    },
    "L2CacheWritebackb": {
        "Read": 3.72729e-11,
        "Write": 3.7854e-11,
        "Search": 3.5083e-11,
    },
}

cache_act_energies["InstCache"]["Read"] = (
    cache_act_energies["InstCacheData"] + cache_act_energies["InstCacheTag"]
)

cache_act_energies["DataCache"]["Read"] = (
    cache_act_energies["DataCacheData"] + cache_act_energies["DataCacheTag"]
)
cache_act_energies["L2Cache"]["Read"] = (
    cache_act_energies["L2CacheData"]
    + cache_act_energies["L2CacheTag"]["Read"]
)
