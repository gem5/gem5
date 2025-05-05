# Copyright (c) 2025 REDS institute of the HEIG-VD
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


# Rename config into _config in PciDevice and a new fields for pxcap
def upgrader(cpt):
    import re

    for sec in cpt.sections():
        # pmcap.pid is a unique option in PciDevice. So it allows to
        # detect any PciDevice.
        if cpt.has_option(sec, "pmcap.pid"):
            cpt.set(sec, "_config.data", cpt.get(sec, "config.data"))
            cpt.remove_option(sec, "config.data")
            pxdc2 = cpt.getint(sec, "pxcap.pxdc2")
            cpt.set(sec, "pxcap.pxdc2", str(pxdc2 & 0xFFFF))
            cpt.set(sec, "pxcap.pxds2", str((pxdc2 >> 16) & 0xFFFF))

            cpt.set(sec, "pxcap.pxscap", "0")
            cpt.set(sec, "pxcap.pxsc", "0")
            cpt.set(sec, "pxcap.pxss", "0")
            cpt.set(sec, "pxcap.pxrcap", "0")
            cpt.set(sec, "pxcap.pxrc", "0")
            cpt.set(sec, "pxcap.pxrs", "0")
            cpt.set(sec, "pxcap.pxlcap2", "0")
            cpt.set(sec, "pxcap.pxlc2", "0")
            cpt.set(sec, "pxcap.pxls2", "0")
            cpt.set(sec, "pxcap.pxscap2", "0")
            cpt.set(sec, "pxcap.pxsc2", "0")
            cpt.set(sec, "pxcap.pxss2", "0")
