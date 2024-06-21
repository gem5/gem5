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

import json
from math import ceil
from pathlib import Path
from typing import (
    List,
    Tuple,
)

from m5.objects import SpatterKernelType
from m5.params import Addr
from m5.util import inform


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

        This file provides four utility function to parse spatter traces:
            parse_kernel: takes a dictionary and returns a tuple of
            delta, count, type, and trace.
            partition_trace: takes an original trace, number of partitions,
            and interleave_size.
            It returns a list of `num_partitions` partitions where each
            partition includes interleaved elements from `original_trace`.
            The elements in the `original_trace` are interleaved with a
            granularity of `interleave_size`.
            unroll_trace: takes an original trace, delta, count, and minimum
            number of elements.
            It will unroll `original_trace` by adding `delta` to the last
            `og_len` (len(`original_trace`)) elements of the trace in steps.
            In each step it will decrement `count` by 1.
            If the logical length of `original_trace` (`og_len` * `count`) is
            smaller than `min_elements`, it allows for filling the trace with
            zeros or elements from the pattern.
            By filling elements from the pattern, the unrolling process goes
            beyond the `count` limit.
            However, it will not decrements `count`.
            prepare_kernels: takes a trace_path, number of cores,
            interleave_size, base_index_addr, and base_value_addr.
            It will return a list of lists of kernels where each list of
            kernels represents a kernel with a length of `num_cores`.
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
            base_index (int): The index from the index array to start from.
            It's most meaningful when used in multi-generator simulations.
            User defined, i.e. spatter traces don't have this field.
            indices_per_stride (int): The number of indices from the index
            array to read from before making a jump of size `stride_size`.
            User defined, i.e. spatter traces don't have this field.
            stride_size (int): The size of the jump to make after reading
            `indices_per_stride` indices from the index array.
            User defined, i.e. spatter traces don't have this field.
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


def parse_kernel(kernel: dict, default_delta=8) -> Tuple[int, int, str, List]:
    """
    Function to parse a kernel from a dictionary. Each Spatter trace is
    represented as a list of dictionaries in JSON. Each dictionary in the list
    represents a kernel. This function will one kernel and return a tuple of
    delta, count, type, and trace.
    Args:
        kernel (dict): A dictionary representing a kernel.
        default_delta (int): The default delta value to use when the delta
        value is not found in the kernel dictionary.
        Returns:
            Tuple[int, int, str, List]: A tuple of delta, count, type,
            and trace extracted from the kernel.
    """
    delta = kernel.get("delta", default_delta)
    if delta < 0:
        inform(
            f"Negative delta found: {delta}. Setting it to {default_delta}. "
            "You can change the default delta value by passing "
            "`default_detla` argument to this function."
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


def unroll_trace(
    original_trace: List,
    delta: int,
    count: int,
    min_elements: int,
    fill_zero=False,
    fill_pattern=False,
):
    """
    Function to unroll a trace by creating replicated elements in the trace.
    This function will add `delta` to the last `og_len` (len(`original_trace`))
    elements of the trace in steps. In each step it will decrement `count`.
    If the logical length of `original_trace` (`og_len` * `count`) is smaller
    than `min_elements`, it allows for filling the trace with zeros or elements
    from the pattern. By filling elements from the pattern, the unrolling
    process goes beyond the `count` limit. However, it will not decrement
    `count`.

    Args:
        original_trace (List): The original trace to unroll.
        delta (int): The delta value as provided from the kernel in JSON.
        count (int): The count value as provided from the kernel in JSON.
        min_elements (int): The minimum number of elements the trace should
        have after unrolling.
        fill_zero (bool): If True, the trace will be filled with zeros when
        the unrolling process runs out of elements from the original trace.
        fill_pattern (bool): If True, the trace will be filled with the pattern
        allowing to go over the `count` limit (from the kernel in JSON) when
        unrolling.
    """
    if fill_zero and fill_pattern:
        raise ValueError(
            f"Only one of fill_zero or fill_pattern can be True. "
            "However, both can be False at the same time."
        )

    if (len(original_trace) * count) < min_elements and (
        not fill_zero and not fill_pattern
    ):
        raise ValueError(
            f"Trace is too small (len(`pattern`) * `count`) < {min_elements}. "
            f"It will not have {min_elements} elements after unrolling. "
            "You can set fill_zero or fill_pattern to True to fill pattern. "
            "fill_zero will fill with zeros when the unrolling process runs "
            "out of elements from the original trace. "
            "fill_pattern will fill with the pattern allowing to go over the "
            "`count` limit (from the kernel in JSON) when unrolling."
        )

    og_len = len(original_trace)
    ret_count = count
    ret_trace = original_trace
    while (len(ret_trace) < min_elements) and ret_count > 1:
        ret_trace += [element + delta for element in ret_trace[-og_len:]]
        ret_count -= 1
    if (len(ret_trace) < min_elements) and fill_zero:
        inform(
            "You have chosen to fill the trace with zero "
            f"until it reaches at least {min_elements} elements."
        )
        ret_trace += [0] * (min_elements - len(ret_trace))
    if (len(ret_trace) < min_elements) and fill_pattern:
        inform(
            "You have chosen to fill the trace with the pattern "
            "(without dectementing count) until it "
            f"reaches at least {min_elements} elements."
        )
        while len(ret_trace) < min_elements:
            ret_trace += [element + delta for element in ret_trace[-og_len:]]
    return ret_count, ret_trace


def partition_trace(original_trace, num_partitions, interleave_size):
    if len(original_trace) < (num_partitions * interleave_size):
        raise ValueError(
            "Trace (`original_trace`) is too small for the "
            "given number of partitions and interleave size. "
            "The trace (`original_trace`) should have at least "
            "`num_partitions` * `interleave_size` elements."
            "It might be due to either the trace being too small "
            "or it being folded too many times. You can solve "
            "this issue by using the `unroll_trace` function. "
        )
    partitions = [[] for _ in range(num_partitions)]
    num_leaves = ceil(len(original_trace) / interleave_size)
    for i in range(num_leaves):
        lower_bound = i * interleave_size
        upper_bound = min(lower_bound + interleave_size, len(original_trace))
        partitions[i % num_partitions] += original_trace[
            lower_bound:upper_bound
        ]
    return partitions


def prepare_kernels(
    trace_path: Path,
    num_cores: int,
    interleave_size: int,
    base_index_addr: Addr,
    base_value_addr: Addr,
) -> List[List[SpatterKernel]]:
    """
    Function to prepare kernels from a spatter trace. It will read the trace
    from the given path and prepare kernels for the given number of cores and
    interleave size. It will partition the trace into `num_cores` partitions
    with `interleave_size` elements in each partition. It will also unroll the
    trace to have at least `num_cores` * `interleave_size` elements in the
    trace using the `unroll_trace` function. In case the trace is too small,
    it will ask `unroll_trace` to fill the trace with elements from the
    pattern. It will return a list of list of kernels where each list of
    kernels represents a kernel with a length of `num_cores`.
    Args:
        trace_path (Path): Path to the spatter trace.
        num_cores (int): Number of cores to partition the trace.
        interleave_size (int): Number of elements to interleave the trace by.
        base_index_addr (Addr): The base address of the index array.
        base_value_addr (Addr): The base address of the value array.
    Returns:
        List[List[SpatterKernel]]: A list of list of kernels where each list
        of kernels represents a kernel with a length of `num_cores`.
    """
    trace_file = trace_path.open("r")
    kernels = json.load(trace_file)
    ret = []
    for i, kernel in enumerate(kernels):
        delta, count, type, og_trace = parse_kernel(kernel)
        new_count, unrolled_trace = unroll_trace(
            og_trace,
            delta,
            count,
            num_cores * interleave_size,
            fill_pattern=True,
        )
        traces = partition_trace(unrolled_trace, num_cores, interleave_size)
        temp = []
        for j, trace in enumerate(traces):
            temp.append(
                SpatterKernel(
                    kernel_id=i,
                    kernel_delta=delta,
                    kernel_count=new_count,
                    kernel_type=type,
                    base_index=j * interleave_size,
                    indices_per_stride=interleave_size,
                    stride_size=interleave_size * num_cores,
                    index_size=4,
                    base_index_addr=base_index_addr,
                    value_size=8,
                    base_value_addr=base_value_addr,
                    kernel_trace=trace,
                )
            )
        ret.append(temp)
    return ret
