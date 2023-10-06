# Copyright (c) 2023 Google LLC
# All rights reserved.
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


def upgrader(cpt):
    # Update the RISC-V pcstate to match the new version of
    # PCState

    for sec in cpt.sections():
        import re

        if re.search(".*processor.*\.core.*\.xc.*", sec):
            if cpt.get(sec, "_rvType", fallback="") == "":
                cpt.set(sec, "_rvType", "1")

            if cpt.get(sec, "_vlenb", fallback="") == "":
                cpt.set(sec, "_vlenb", "32")

            if cpt.get(sec, "_vtype", fallback="") == "":
                cpt.set(sec, "_vtype", str(1 << 63))

            if cpt.get(sec, "_vl", fallback="") == "":
                cpt.set(sec, "_vl", "0")

            if cpt.get(sec, "_compressed", fallback="") == "":
                cpt.set(sec, "_compressed", "false")
