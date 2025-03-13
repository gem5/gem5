# Copyright (c) 2021 The Regents of The University of California
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

import enum
import pathlib
from datetime import datetime
from typing import (
    Dict,
    List,
    Optional,
    Tuple,
    Union,
)

from .group import Group
from .statistic import Statistic
from .timeconversion import TimeConversion


class SimStat(Group):
    """
    Contains all the statistics for a given simulation.
    """

    def __init__(
        self,
        creation_time: Optional[datetime] = None,
        time_conversion: Optional[TimeConversion] = None,
        simulated_begin_time: Optional[Union[int, float]] = None,
        simulated_end_time: Optional[Union[int, float]] = None,
        **kwargs: Dict[str, Union[Group, Statistic, List[Group]]],
    ):
        super().__init__(
            creation_time=creation_time,
            time_conversion=time_conversion,
            simulated_begin_time=simulated_begin_time,
            simulated_end_time=simulated_end_time,
            **kwargs,
        )

    @classmethod
    def dictFromFile(cls, filename: pathlib.Path) -> Tuple[Dict, pathlib.Path]:
        """
        Parse the statistics provided by gem5 in the stats.txt file
        and return them as a dict of 'useful' python values.
        """

        class LineType(enum.Enum):
            """
            Tiny enum to help classify the stat entries
            """

            VALUE = 0
            DIST = 1
            HEADER = 2
            UNKNOWN = 3

        def check(line: str) -> Tuple[str, List[float], LineType]:
            """
            Verify and transform a statistics line into a useful
            collection of values, namely the statistic, values and
            line type
            """

            def get_type(line: List[str]) -> LineType:
                """
                Check if this line reports a single value, a distribution
                or is a header. If it is not one of these, report Unknown
                """
                # TODO this is a pretty flimsy method of classification
                if len(line) == 2:
                    return LineType.VALUE
                elif len(line) == 4:
                    return LineType.DIST
                elif len(line) < 2:
                    return LineType.HEADER
                else:
                    return LineType.UNKNOWN

            def transform(val: List[str]) -> List[float]:
                """
                Transform the string into a float
                """
                # TODO this should depend on the type of the line, but
                #  for now this provides a good enough default
                return [float(x.split("%")[0]) for x in val]

            line_elems = line.split(" ")
            clean = list(filter(lambda x: x != "", line_elems))

            # remove the comment (all things after #)
            is_comment = False
            super_clean = []
            for elem in clean:
                if elem == "#" or elem.startswith("-"):
                    is_comment = True

                # print(f"{elem.strip()}: {is_comment}")
                if not is_comment:
                    super_clean.append(elem)

            try:
                return (
                    super_clean[0],
                    transform(super_clean[1:]),
                    get_type(super_clean),
                )
            except IndexError:
                return "", [], get_type(super_clean)

        def parse_stats(filename: pathlib.Path) -> Tuple[Dict, pathlib.Path]:
            """
            Parse the stats file into a python dictionary
            """
            stats = {}
            with open(filename) as f:
                for line in f:
                    element, value, t = check(line)
                    if t == LineType.VALUE:
                        dictionary_entries = element.split(".")
                        final_entry = dictionary_entries[-1].split("::")

                        true_entry = dictionary_entries[:-1] + final_entry
                        stats = generate_nested_dict(stats, true_entry, value)

                    elif t == LineType.DIST:
                        pass
                    elif t == LineType.HEADER:
                        continue
                    elif t == LineType.UNKNOWN:
                        print(
                            f"Unknown type encountered in line: {line}; continuing"
                        )
                    else:
                        exit("Failed to parse line")

            return stats, filename

        def generate_nested_dict(
            init_dict: Dict, nesting_list: List[str], values: List[float]
        ):
            """
            The jank one... This function, given a list of keys, will generate the corresponding
            dictionary entry with values as the value.

            For example, if nesting_list is ['a', 'b', 'c'] and values is [1, 2, 3] then
            we will provide dictionary {'a': {'b': {'c': [1, 2, 3]}}}. If the entries exist
            in the init_dict, we will use those entries and only add keys that do not yet
            exist.
            """
            init_dict_ptr = init_dict
            for i, e in enumerate(nesting_list):
                try:
                    if init_dict[e] is not None:
                        pass
                except KeyError:
                    init_dict[e] = {}

                if i == len(nesting_list) - 1:
                    init_dict[e] = values
                else:
                    init_dict = init_dict[e]

            return init_dict_ptr

        return parse_stats(filename)
