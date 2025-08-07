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


# Seperate config data into their own index
def upgrader(cpt):
    import re

    def bytesToX(arr, offset, size):
        val = 0

        for i in range(size):
            val += int(arr[offset + i]) << (i * 8)

        return str(val)

    def addCapabilitySection(cpt, sec, suffix, names):
        new_sec = f"{sec}.{suffix}"
        cpt.add_section(new_sec)

        for old_name, new_name in names.items():
            cpt.set(new_sec, new_name, cpt.get(sec, f"{suffix}.{old_name}"))

    def removeCapabilities(cpt, sec, suffix, names):
        for old_name, new_name in names.items():
            cpt.remove_option(sec, f"{suffix}.{old_name}")

    for sec in cpt.sections():
        # pmcap.pid is a unique option in PciDevice. So it allows to
        # detect any PciDevice.
        if cpt.has_option(sec, "pmcap.pid"):
            data = cpt.get(sec, "_config.data").split(" ")

            # Convert data bytes array into individual configuration
            cpt.set(sec, "vendorId", bytesToX(data, 0x0, 2))
            cpt.set(sec, "deviceId", bytesToX(data, 0x2, 2))
            cpt.set(sec, "command", bytesToX(data, 0x4, 2))
            cpt.set(sec, "status", bytesToX(data, 0x6, 2))
            cpt.set(sec, "revision", bytesToX(data, 0x8, 1))
            cpt.set(sec, "progIF", bytesToX(data, 0x9, 1))
            cpt.set(sec, "subClassCode", bytesToX(data, 0xA, 1))
            cpt.set(sec, "classCode", bytesToX(data, 0xB, 1))
            cpt.set(sec, "cacheLineSize", bytesToX(data, 0xC, 1))
            cpt.set(sec, "latencyTimer", bytesToX(data, 0xD, 1))
            cpt.set(sec, "headerType", bytesToX(data, 0xE, 1))
            cpt.set(sec, "bist", bytesToX(data, 0xF, 1))
            cpt.set(sec, "capabilityPtr", bytesToX(data, 0x34, 1))
            cpt.set(sec, "interruptLine", bytesToX(data, 0x3C, 1))
            cpt.set(sec, "interruptPin", bytesToX(data, 0x3D, 1))

            # Header type dependent configuration
            if int(data[0xE]) == 0x0:
                bar_count = 6
                cpt.set(sec, "cardbusCIS", bytesToX(data, 0x28, 4))
                cpt.set(sec, "subsystemVendorID", bytesToX(data, 0x2C, 2))
                cpt.set(sec, "subsystemID", bytesToX(data, 0x2E, 2))
                cpt.set(sec, "expansionROM", bytesToX(data, 0x30, 4))
                cpt.set(sec, "minimumGrant", bytesToX(data, 0x3E, 1))
                cpt.set(sec, "maximumLatency", bytesToX(data, 0x3F, 1))
            elif int(data[0xE]) == 0x1:
                bar_count = 2
                cpt.set(sec, "primaryBusNum", bytesToX(data, 0x18, 1))
                cpt.set(sec, "secondaryBusNum", bytesToX(data, 0x19, 1))
                cpt.set(sec, "subordinateBusNum", bytesToX(data, 0x1A, 1))
                cpt.set(sec, "secondaryLatencyTimer", bytesToX(data, 0x1B, 1))
                cpt.set(sec, "ioBase", bytesToX(data, 0x1C, 1))
                cpt.set(sec, "ioLimit", bytesToX(data, 0x1D, 1))
                cpt.set(sec, "secondaryStatus", bytesToX(data, 0x1E, 2))
                cpt.set(sec, "memBase", bytesToX(data, 0x20, 2))
                cpt.set(sec, "memLimit", bytesToX(data, 0x22, 2))
                cpt.set(sec, "prefetchMemBase", bytesToX(data, 0x24, 2))
                cpt.set(sec, "prefetchMemLimit", bytesToX(data, 0x26, 2))
                cpt.set(sec, "prefetchBaseUpper", bytesToX(data, 0x28, 4))
                cpt.set(sec, "prefetchLimitUpper", bytesToX(data, 0x2C, 4))
                cpt.set(sec, "ioBaseUpper", bytesToX(data, 0x30, 2))
                cpt.set(sec, "ioLimitUpper", bytesToX(data, 0x32, 2))
                cpt.set(sec, "expansionROM", bytesToX(data, 0x38, 4))
                cpt.set(sec, "bridgeControl", bytesToX(data, 0x3E, 2))
            else:
                raise ValueError(f"Unknown header type ({sec})")

            for i in range(bar_count):
                cpt.set(sec, f"bar_{i}", bytesToX(data, 0x10 + i * 4, 4))

            cpt.remove_option(sec, "_config.data")

            # Create a new section for each PCI capabilities
            pmcap_names = {
                "pc": "pmc",
                "pmcs": "pmcs",
            }
            if cpt.getint(sec, "pmcap.pid") != 0x0:
                addCapabilitySection(cpt, sec, "pmcap", pmcap_names)
            cpt.remove_option(sec, "pmcap.pid")
            removeCapabilities(cpt, sec, "pmcap", pmcap_names)

            msicap_names = {
                "mc": "mc",
                "ma": "ma",
                "mua": "mua",
                "md": "md",
                "mmask": "mmask",
                "mpend": "mpend",
            }
            if cpt.getint(sec, "msicap.mid") != 0x0:
                addCapabilitySection(cpt, sec, "msicap", msicap_names)
            cpt.remove_option(sec, "msicap.mid")
            removeCapabilities(cpt, sec, "msicap", msicap_names)

            msixcap_names = {
                "mxc": "mxc",
                "mtab": "mtab",
                "mpba": "mpba",
            }
            if cpt.getint(sec, "msixcap.mxid") != 0x0:
                addCapabilitySection(cpt, sec, "msixcap", msixcap_names)
            cpt.remove_option(sec, "msixcap.mxid")
            removeCapabilities(cpt, sec, "msixcap", msixcap_names)

            pxcap_names = {
                "pxcap": "pxcap",
                "pxdcap": "pxdcap",
                "pxdc": "pxdc",
                "pxds": "pxds",
                "pxlcap": "pxlcap",
                "pxlc": "pxlc",
                "pxls": "pxls",
                "pxscap": "pxscap",
                "pxsc": "pxsc",
                "pxss": "pxss",
                "pxrc": "pxrc",
                "pxrcap": "pxrcap",
                "pxrs": "pxrs",
                "pxdcap2": "pxdcap2",
                "pxdc2": "pxdc2",
                "pxds2": "pxds2",
                "pxlcap2": "pxlcap2",
                "pxlc2": "pxlc2",
                "pxls2": "pxls2",
                "pxscap2": "pxscap2",
                "pxsc2": "pxsc2",
                "pxss2": "pxss2",
            }
            if cpt.getint(sec, "pxcap.pxid") != 0x0:
                addCapabilitySection(cpt, sec, "pxcap", pxcap_names)
            cpt.remove_option(sec, "pxcap.pxid")
            removeCapabilities(cpt, sec, "pxcap", pxcap_names)
