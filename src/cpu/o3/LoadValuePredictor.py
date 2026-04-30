# Copyright (c) 2026 NITC Calicut Research
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

"""
Python SimObject wrapper for the Load Value Predictor module.

This file exposes the three configurable knobs (numEntries,
confidenceThreshold, instShiftAmount) to the gem5 Python configuration
system so they can be overridden from a simulation script without
recompiling C++.
"""

from m5.params import *
from m5.SimObject import SimObject


class LoadValuePredictor(SimObject):
    type = "LoadValuePredictor"
    cxx_class = "gem5::o3::LoadValuePredictor"
    cxx_header = "cpu/o3/lvp.hh"

    # Number of entries in the PC-tagged prediction table.
    # Must be a power of two (direct-mapped via bitmask).
    numEntries = Param.Unsigned(1024, "Number of LVP table entries")

    # How many identical consecutive observations are required before the
    # predictor is "confident" enough to speculatively forward the value.
    # 250 is the empirical value reverse-engineered from Apple M3 in the
    # FLOP paper (Kim et al., USENIX Security 2025).
    confidenceThreshold = Param.Unsigned(
        250, "Consecutive identical values needed before issuing a prediction"
    )

    # Right-shift applied to the PC before indexing into the table.
    # For ARM (AArch64), all instructions are 4-byte aligned so the bottom
    # 2 bits are always zero — shifting them away improves distribution.
    instShiftAmount = Param.Unsigned(
        2, "PC right-shift before table index (2 for ARM 32-bit instructions)"
    )
