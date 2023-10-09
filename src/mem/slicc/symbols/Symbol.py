# Copyright (c) 2022 Arm Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
# Copyright (c) 2009 The Hewlett-Packard Development Company
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
from slicc.util import PairContainer


class Symbol(PairContainer):
    def __init__(self, symtab, ident, location, pairs=None):
        super().__init__()

        from slicc.util import Location
        from slicc.symbols import SymbolTable

        if not isinstance(symtab, SymbolTable):
            raise AttributeError
        if not isinstance(ident, str):
            raise AttributeError
        if not isinstance(location, Location):
            raise AttributeError

        self.symtab = symtab
        self.ident = ident
        self.location = location
        if pairs:
            self.pairs.update(getattr(pairs, "pairs", pairs))
        if "short" not in self:
            self["short"] = self.ident
        self.used = False

    def __repr__(self):
        return f"[Symbol: {self.ident}]"

    def __str__(self):
        return str(self.ident)

    def __setitem__(self, key, value):
        if key in self.pairs:
            self.warning(
                "Pair key '%s' re-defined. new: '%s' old: '%s'",
                key,
                value,
                self.pairs[key],
            )
        super().__setitem__(key, value)

    @property
    def short(self):
        return self["short"]

    @property
    def desc(self):
        # Allow Symbols with no description: return an empty string.
        if "desc" not in self:
            return ""
        else:
            return self["desc"]

    def error(self, message, *args):
        self.location.error(message, *args)

    def warning(self, message, *args):
        self.location.warning(message, *args)

    def writeHTMLFiles(self, path):
        pass


__all__ = ["Symbol"]
