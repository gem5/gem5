# Copyright (c) 2026 Sungkyunkwan University
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

import re
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from typing import (
    Iterable,
    List,
    Optional,
)
from typing import OrderedDict as OrderedDictType
from typing import (
    Tuple,
    Union,
)

_BEGIN_RE = re.compile(
    r"^---------- Begin Simulation Statistics(?: : (?P<message>.*))? ----------$"
)
_END_LINE = "---------- End Simulation Statistics   ----------"
_INTEGER_RE = re.compile(r"^[+-]?\d+$")

StatsValue = Union[int, float]


class StatsParseError(ValueError):
    pass


@dataclass
class StatsDump:
    stats: OrderedDictType[str, StatsValue]
    line_number: int
    message: str = ""

    def __getitem__(self, stat_name: str) -> StatsValue:
        return self.stats[stat_name]


def parse_stats_file(path: Union[Path, str]) -> List[StatsDump]:
    with open(path, encoding="utf-8") as stats_file:
        return parse_stats_text(stats_file)


def parse_stats_text(lines: Union[Iterable[str], str]) -> List[StatsDump]:
    if isinstance(lines, str):
        lines = lines.splitlines()

    dumps: List[StatsDump] = []
    current: Optional[StatsDump] = None

    for line_number, line in enumerate(lines, start=1):
        stripped = line.strip()

        if not stripped or stripped.startswith("#"):
            continue

        if _is_begin_line(stripped):
            if current is not None:
                raise StatsParseError(
                    f"line {line_number}: found a new stats dump before "
                    "the previous dump ended"
                )
            current = StatsDump(
                stats=OrderedDict(),
                line_number=line_number,
                message=_parse_message(stripped),
            )
            continue

        if stripped == _END_LINE:
            if current is None:
                raise StatsParseError(
                    f"line {line_number}: found stats dump end before begin"
                )
            dumps.append(current)
            current = None
            continue

        if current is None:
            continue

        parsed = _parse_stat_line(stripped, line_number)
        if parsed is None:
            continue

        name, value = parsed
        if name in current.stats:
            raise StatsParseError(
                f"line {line_number}: duplicate stat name '{name}'"
            )
        current.stats[name] = value

    if current is not None:
        raise StatsParseError(
            f"line {current.line_number}: stats dump was not terminated"
        )

    return dumps


def _is_begin_line(line: str) -> bool:
    return _BEGIN_RE.match(line) is not None


def _parse_message(line: str) -> str:
    match = _BEGIN_RE.match(line)
    if match is None:
        return ""
    return match.group("message") or ""


def _parse_stat_line(
    line: str, line_number: int
) -> Optional[Tuple[str, StatsValue]]:
    parts = line.split(maxsplit=2)
    if len(parts) < 2:
        raise StatsParseError(f"line {line_number}: missing stat value")

    name, value_text = parts[0], parts[1]
    if value_text == "|":
        # Oneline vector/dist stats are valid text output, but they do not
        # encode a single stat-name/value pair. Do not synthesize names here.
        return None

    return name, _parse_value(value_text, line_number, name)


def _parse_value(
    value_text: str, line_number: int, stat_name: str
) -> StatsValue:
    if _INTEGER_RE.match(value_text):
        return int(value_text)

    try:
        return float(value_text)
    except ValueError as error:
        raise StatsParseError(
            f"line {line_number}: invalid stat value '{value_text}' "
            f"for '{stat_name}'"
        ) from error
