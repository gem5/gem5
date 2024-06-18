# Copyright (c) 2024 The Regents of the University of California
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

from math import ceil
from typing import (
    List,
    Tuple,
)

from m5.objects import SpatterKernelType
from m5.params import Addr
from m5.util import inform


def parse_kernel(kernel: dict, default_delta=8) -> Tuple[int, int, str, List]:
    delta = kernel.get("delta", default_delta)
    if delta < 0:
        inform(
            f"Negative delta found: {delta}. Setting it to {default_delta}."
        )
        delta = default_delta
    count = kernel.get("count", 1)
    type = kernel.get("kernel", None)
    if type is None:
        raise ValueError(f"Keyword 'kernel' not found.")
    type = SpatterKernelType(type.lower())
    trace = kernel.get("pattern", [])
    if len(trace) == 0:
        raise ValueError(f"Empty 'pattern' found.")
    return (delta, count, type, trace)


def partition_trace(original_trace, num_partitions, interleave_size):
    partitions = [[] for _ in range(num_partitions)]
    num_leaves = ceil(len(original_trace) / interleave_size)
    for i in range(num_leaves):
        lower_bound = i * interleave_size
        upper_bound = min(lower_bound + interleave_size, len(original_trace))
        partitions[i % num_partitions] += original_trace[
            lower_bound:upper_bound
        ]
    return partitions


class SpatterKernel:
    """This class encapsulates one kernel in a spatter trace.
        A spatter trace is represented with a json file.
        An example of a spatter trace can be found here:
    https://github.com/hpcgarage/spatter/blob/main/standard-suite/app-traces/amg.json
        Each trace may have multiple kernels.
        Each kernel represents a code execution like below
            for (int iteration = 0; iteration < count; iteration++)
            {
                for (int i = 0; i < N; i++) {
                    value[index[i] + iteration * delta] = rand(); // kernel: scatter
                    // OR
                    sum += value[index[i] + iteration * delta]; // kernel: gather
                }
            }
        Where `delta` and `count` are fields in each kernel.
        `kernel` is another field that determines whether the accesses to value
        are loads or stores.
        The field `pattern` stores the index array.

        This file provides two utility function to parse spatter traces:
            parse_kernel: takes a dictionary and returns a tuple of
            delta, count, type, and trace.
            partition_trace: takes the original trace, number of partitions,
            and interleave_size.
            It returns a list of `num_partitions` partitions where each partition
            is an list including interleaved elements from `original_trace`.
            The elements in the `original_trace` are interleaved with a
            granularity of `interleave_size`.
        The code snippet below shows how to use these functions to create kernels.
            generator = SpatterGenerator(num_cores)

            with open(trace_path, "r") as trace_file:
                kernels = json.load(trace_file)

            for i, kernel in enumerate(kernels):
                delta, count, type, og_trace = parse_kernel(kernel)
                traces = partition_trace(og_trace, num_cores, 128)
                kernels = [SpatterKernel(
                                        kernel_id=i,
                                        kernel_delta=delta,
                                        kernel_count=count,
                                        kernel_type=type,
                                        kernel_trace=trace,
                                        index_size=4,
                                        base_index_addr=0,
                                        value_size=8,
                                        base_value_addr=0x400000000
                                        )
                            for trace in traces
                            ]
                generator.add_kernel(kernels)

        Args:
            kernel_id (int): The ID of the kernel.
            User defined, i.e. spatter traces don't have this field.
            It's used to identify the kernel in the simulation.
            kernel_delta (int): The delta value of the kernel.
            `delta` from spatter trace.
            kernel_count (int): The count value of the kernel.
            `count` from spatter trace.
            kernel_type (SpatterKernelType): The type of the kernel.
            `kernel` from spatter trace.
            kernel_trace (List[int]): The elements of the `index` array.
            `pattern` from spatter trace.
            index_size (int): The size of elements in `index`.
            User defined, i.e. spatter traces don't have this field.
            It represents the size of elements in the `index` array in code above.
            base_index_addr (Addr): The base address of the index.
            User defined, i.e. spatter traces don't have this field.
            It represents the pointer to the `index` array in the code above.
            value_size (int): The size of elements in `value`.
            User defined, i.e. spatter traces don't have this field.
            It represents the size of elements in the `value` array in code above.
            base_value_addr (Addr): The base address of the value.
            User defined, i.e. spatter traces don't have this field.
            It represents the pointer to the `value` array in the code above.
    """

    def __init__(
        self,
        kernel_id: int,
        kernel_delta: int,
        kernel_count: int,
        kernel_type: SpatterKernelType,
        base_index: int,
        indices_per_stride: int,
        stride_size: int,
        index_size: int,
        base_index_addr: Addr,
        value_size: int,
        base_value_addr: Addr,
        kernel_trace: List[int],
        fix_empty_trace: bool = False,
    ):
        self._id = kernel_id
        self._delta = kernel_delta
        self._count = kernel_count
        self._type = kernel_type
        self._base_index = base_index
        self._indices_per_stride = indices_per_stride
        self._stride_size = stride_size
        self._index_size = index_size
        self._base_index_addr = base_index_addr
        self._value_size = value_size
        self._base_value_addr = base_value_addr
        self._trace = kernel_trace

        if fix_empty_trace and len(kernel_trace) == 0:
            inform(
                "Empty trace found. Fixing it by adding a dummy element. "
                "Also setting delta to 0 and count to 1.",
            )
            self._trace = [0]
            self._delta = 0
            self._count = 1

    def empty(self):
        return len(self._trace) == 0

    def cxx_call_args(self):
        return [
            self._id,
            self._delta,
            self._count,
            self._type.getValue(),
            self._base_index,
            self._indices_per_stride,
            self._stride_size,
            self._index_size,
            self._base_index_addr,
            self._value_size,
            self._base_value_addr,
            self._trace,
        ]

    def __str__(self):
        return (
            f"SpatterKernel(id={self._id}, delta={self._delta}, "
            f"count={self._count}, type={self._type}, "
            f"trace[:8]={self._trace[:8]}"
        )
