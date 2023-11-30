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

import json
from datetime import datetime
from typing import (
    IO,
    Any,
    Dict,
    List,
    Union,
)

from .storagetype import StorageType


class SerializableStat:
    """
    Classes which inherit from SerializableStat can be serialized as JSON
    output.

    Usage
    -----
    ```
    import m5.pystats.gem5stats as gem5stats

    simstat = gem5stats.get_simstat(root)
    print(simstat.dumps())
    ```
    """

    def to_json(self) -> Dict:
        """
        Translates the current object into a JSON dictionary.

        Returns
        -------
        Dict
            The JSON dictionary.
        """

        model_dct = {}
        for key, value in self.__dict__.items():
            new_value = self.__process_json_value(value)
            model_dct[key] = new_value
        return model_dct

    def __process_json_value(
        self, value: Any
    ) -> Union[str, int, float, Dict, List, None]:
        """
        Translate values into a value which can be handled by the Python stdlib
        JSON package.

        Parameters
        ----------
        value: Any
            The value to be translated.

        Returns
        -------
        Union[str,int,float,Dict,List]
            A value which can be handled by the Python stdlib JSON package.
        """

        if isinstance(value, SerializableStat):
            return value.to_json()
        elif isinstance(value, (str, int, float)):
            return value
        elif isinstance(value, datetime):
            return value.replace(microsecond=0).isoformat()
        elif isinstance(value, list):
            return [self.__process_json_value(v) for v in value]
        elif isinstance(value, StorageType):
            return str(value.name)

        return None

    def dumps(self, **kwargs) -> str:
        """
        This function mirrors the Python stdlib JSON module method
        `json.dumps`. It is used to obtain the gem5 statistics output to a
        JSON string.

        Parameters
        ----------
        root: Root
            The root of the simulation.

        kwargs: Dict[str, Any]
            Additional parameters to be passed to the `json.dumps` method.

        Returns
        -------
        str
            A string of the gem5 Statistics in a JSON format.


        Usage Example
        -------------
        ```
        import m5.pystats.gem5stats as gem5stats

        simstat = gem5stats.get_simstat(root)
        print(simstat.dumps(indent=6))
        ```

        The above will print the simulation statistic JSON string. The
        indentation will be 6 (by default the indentation is 4).
        """

        # Setting the default indentation to something readable.
        if "indent" not in kwargs:
            kwargs["indent"] = 4

        return json.dumps(obj=self.to_json(), **kwargs)

    def dump(self, fp: IO[str], **kwargs) -> None:
        """
        This function mirrors the Python stdlib JSON module method
        `json.dump`. The root of the simulation is passed, and the JSON is
        output to the specified.


        Parameters
        ----------
        fp: IO[str]
            The Text IO stream to output the JSON to.

        **kwargs:
            Additional parameters to be passed to the ``json.dump`` method.

        Usage
        -----
        ```
        import m5.pystats.gem5stats as gem5stats

        simstat = gem5stats.get_simstat(root)
        with open("test.json") as f:
            simstat.dump(fp=f, indent=6)
        ```

        The above will dump the json output to the 'test.json' file. The
        indentation will be of 6 (by default the indentation is 4).
        """

        # Setting the default indentation to something readable.
        if "indent" not in kwargs:
            kwargs["indent"] = 4

        json.dump(obj=self.to_json(), fp=fp, **kwargs)
