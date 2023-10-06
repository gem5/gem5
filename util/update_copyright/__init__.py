# Copyright (c) 2020 ARM Limited
# All rights reserved
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
"""
Utilities to parse and modify copyright headers in gem5 source.
"""
import re

org_alias_map = {
    "arm": b"ARM Limited",
    "uc": b"The Regents of the University of California",
}

_update_copyright_year_regexp = re.compile(b"(.*?)([0-9]+)$")


def _update_copyright_years(m, cur_year, org_bytes):
    """
    Does e.g.: b'2016, 2018-2019' -> b'2016, 2018-2020'.

    :param m: match containing only the years part of the string
    :type m: re.Match
    :param cur_year: the current year to update the copyright to
    :type cur_year: int
    :return: the new years part of the string
    :rtype: bytes
    """
    global _update_copyright_year_regexp
    cur_year_bytes = str(cur_year).encode()
    m = _update_copyright_year_regexp.match(m.group(1))
    years_prefix = m.group(1)
    old_year_bytes = m.group(2)
    old_year = int(old_year_bytes.decode())
    if old_year == cur_year:
        new_years_string = old_year_bytes
    elif old_year == cur_year - 1:
        if len(years_prefix) > 0 and years_prefix[-1:] == b"-":
            new_years_string = cur_year_bytes
        else:
            new_years_string = old_year_bytes + b"-" + cur_year_bytes
    else:
        new_years_string = old_year_bytes + b", " + cur_year_bytes
    new_years_string = years_prefix + new_years_string
    return b" Copyright (c) %b %b\n" % (new_years_string, org_bytes)


def update_copyright(data, cur_year, org_bytes):
    update_copyright_regexp = re.compile(
        b" Copyright \\(c\\) ([0-9,\\- ]+) " + org_bytes + b"\n",
        re.IGNORECASE,
    )
    return update_copyright_regexp.sub(
        lambda m: _update_copyright_years(m, cur_year, org_bytes),
        data,
        count=1,
    )
