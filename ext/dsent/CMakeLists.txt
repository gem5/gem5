# Copyright (c) 2014 Mark D. Hill and David A. Wood
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

cmake_minimum_required (VERSION 2.6)

project (DSENT)

set(SRCS
    DSENT.cc
    interface.cc
    libutil/Calculator.cc
    libutil/Config.cc
    libutil/Exception.cc
    libutil/Log.cc
    libutil/MathUtil.cc
    libutil/String.cc
    model/ElectricalModel.cc
    model/EventInfo.cc
    model/Model.cc
    model/ModelGen.cc
    model/OpticalModel.cc
    model/PortInfo.cc
    model/TransitionInfo.cc
    model/electrical/BarrelShifter.cc
    model/electrical/BroadcastHTree.cc
    model/electrical/DFFRAM.cc
    model/electrical/Decoder.cc
    model/electrical/DemuxTreeDeserializer.cc
    model/electrical/MatrixArbiter.cc
    model/electrical/Multiplexer.cc
    model/electrical/MultiplexerCrossbar.cc
    model/electrical/MuxTreeSerializer.cc
    model/electrical/OR.cc
    model/electrical/RepeatedLink.cc
    model/electrical/RippleAdder.cc
    model/electrical/SeparableAllocator.cc
    model/electrical/TestModel.cc
    model/electrical/router/Router.cc
    model/electrical/router/RouterInputPort.cc
    model/electrical/router/RouterSwitchAllocator.cc
    model/network/ElectricalClos.cc
    model/network/ElectricalMesh.cc
    model/network/PhotonicClos.cc
    model/optical/GatedLaserSource.cc
    model/optical/LaserSource.cc
    model/optical/OpticalLinkBackendRx.cc
    model/optical/OpticalLinkBackendTx.cc
    model/optical/OpticalTestModel.cc
    model/optical/RingDetector.cc
    model/optical/RingFilter.cc
    model/optical/RingModulator.cc
    model/optical/SWMRLink.cc
    model/optical/SWSRLink.cc
    model/optical/ThrottledLaserSource.cc
    model/optical_graph/OpticalDetector.cc
    model/optical_graph/OpticalFilter.cc
    model/optical_graph/OpticalGraph.cc
    model/optical_graph/OpticalLaser.cc
    model/optical_graph/OpticalModulator.cc
    model/optical_graph/OpticalNode.cc
    model/optical_graph/OpticalWaveguide.cc
    model/optical_graph/OpticalWavelength.cc
    model/std_cells/ADDF.cc
    model/std_cells/AND2.cc
    model/std_cells/BUF.cc
    model/std_cells/CellMacros.cc
    model/std_cells/DFFQ.cc
    model/std_cells/INV.cc
    model/std_cells/LATQ.cc
    model/std_cells/MUX2.cc
    model/std_cells/NAND2.cc
    model/std_cells/NOR2.cc
    model/std_cells/OR2.cc
    model/std_cells/StdCell.cc
    model/std_cells/StdCellLib.cc
    model/std_cells/XOR2.cc
    model/timing_graph/ElectricalDelay.cc
    model/timing_graph/ElectricalDriver.cc
    model/timing_graph/ElectricalDriverMultiplier.cc
    model/timing_graph/ElectricalLoad.cc
    model/timing_graph/ElectricalNet.cc
    model/timing_graph/ElectricalTimingNode.cc
    model/timing_graph/ElectricalTimingOptimizer.cc
    model/timing_graph/ElectricalTimingTree.cc
    tech/TechModel.cc
    util/Constants.cc
    util/Result.cc
)

# add the library
add_library(dsent MODULE ${SRCS})

SET(PYTHON_CONFIG "python-config")

# add the binary tree to the search path for include files
include_directories("${CMAKE_SOURCE_DIR}" "`${PYTHON_CONFIG} --includes`")
EXEC_PROGRAM(${PYTHON_CONFIG} ARGS --cflags OUTPUT_VARIABLE PYTHON_CFLAGS)
EXEC_PROGRAM(${PYTHON_CONFIG} ARGS --ldflags OUTPUT_VARIABLE PYTHON_LFLAGS)

set_target_properties(dsent PROPERTIES
        PREFIX ""
        COMPILE_FLAGS "${PYTHON_CFLAGS} -std=c++11"
        LINK_FLAGS "${PYTHON_LFLAGS}")
