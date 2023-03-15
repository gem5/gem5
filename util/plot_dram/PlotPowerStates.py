# Copyright (c) 2017 ARM Limited
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

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np
import os

# global results dict
results = {}
idleResults = {}

# global vars for bank utilisation and seq_bytes values swept in the experiment
bankUtilValues = []
seqBytesValues = []
delayValues = []

# settings for 3 values of bank util and 3 values of seq_bytes
stackHeight = 6.0
stackWidth = 18.0
barWidth = 0.5
plotFontSize = 18

States = ["IDLE", "ACT", "REF", "ACT_PDN", "PRE_PDN", "SREF"]

EnergyStates = [
    "ACT_E",
    "PRE_E",
    "READ_E",
    "REF_E",
    "ACT_BACK_E",
    "PRE_BACK_E",
    "ACT_PDN_E",
    "PRE_PDN_E",
    "SREF_E",
]

StackColors = {
    "IDLE": "black",  # time spent in states
    "ACT": "lightskyblue",
    "REF": "limegreen",
    "ACT_PDN": "crimson",
    "PRE_PDN": "orange",
    "SREF": "gold",
    "ACT_E": "lightskyblue",  # energy of states
    "PRE_E": "black",
    "READ_E": "white",
    "REF_E": "limegreen",
    "ACT_BACK_E": "lightgray",
    "PRE_BACK_E": "gray",
    "ACT_PDN_E": "crimson",
    "PRE_PDN_E": "orange",
    "SREF_E": "gold",
}

StatToKey = {
    "system.mem_ctrls_0.actEnergy": "ACT_E",
    "system.mem_ctrls_0.preEnergy": "PRE_E",
    "system.mem_ctrls_0.readEnergy": "READ_E",
    "system.mem_ctrls_0.refreshEnergy": "REF_E",
    "system.mem_ctrls_0.actBackEnergy": "ACT_BACK_E",
    "system.mem_ctrls_0.preBackEnergy": "PRE_BACK_E",
    "system.mem_ctrls_0.actPowerDownEnergy": "ACT_PDN_E",
    "system.mem_ctrls_0.prePowerDownEnergy": "PRE_PDN_E",
    "system.mem_ctrls_0.selfRefreshEnergy": "SREF_E",
}
# Skipping write energy, the example script issues 100% reads by default
# 'system.mem_ctrls_0.writeEnergy' : "WRITE"


def plotLowPStates(
    plot_dir, stats_fname, bank_util_list, seqbytes_list, delay_list
):
    """
    plotLowPStates generates plots by parsing statistics output by the DRAM
    sweep simulation described in the the configs/dram/low_power_sweep.py
    script.

    The function outputs eps format images for the following plots
    (1) time spent in the DRAM Power states as a stacked bar chart
    (2) energy consumed by the DRAM Power states as a stacked bar chart
    (3) idle plot for the last stats dump corresponding to an idle period

    For all plots, the time and energy values of the first rank (i.e. rank0)
    are plotted because the way the script is written means stats across ranks
    are similar.

    @param plot_dir: the dir to output the plots
    @param stats_fname: the stats file name of the low power sweep sim
    @param bank_util_list: list of bank utilisation values (e.g. [1, 4, 8])
    @param seqbytes_list: list of seq_bytes values (e.g. [64, 456, 512])
    @param delay_list: list of itt max multipliers (e.g. [1, 20, 200])

    """
    stats_file = open(stats_fname, "r")

    global bankUtilValues
    bankUtilValues = bank_util_list

    global seqBytesValues
    seqBytesValues = seqbytes_list

    global delayValues
    delayValues = delay_list
    initResults()

    # throw away the first two lines of the stats file
    stats_file.readline()
    stats_file.readline()  # the 'Begin' line

    #######################################
    # Parse stats file and gather results
    ########################################

    for delay in delayValues:
        for bank_util in bankUtilValues:
            for seq_bytes in seqBytesValues:

                for line in stats_file:
                    if "Begin" in line:
                        break

                    if len(line.strip()) == 0:
                        continue

                    #### state time values ####
                    if "system.mem_ctrls_0.memoryStateTime" in line:
                        # remove leading and trailing white spaces
                        line = line.strip()
                        # Example format:
                        # 'system.mem_ctrls_0.memoryStateTime::ACT    1000000'
                        statistic, stime = line.split()[0:2]
                        # Now grab the state, i.e. 'ACT'
                        state = statistic.split("::")[1]
                        # store the value of the stat in the results dict
                        results[delay][bank_util][seq_bytes][state] = int(
                            stime
                        )
                    #### state energy values ####
                    elif line.strip().split()[0] in list(StatToKey.keys()):
                        # Example format:
                        # system.mem_ctrls_0.actEnergy                 35392980
                        statistic, e_val = line.strip().split()[0:2]
                        senergy = int(float(e_val))
                        state = StatToKey[statistic]
                        # store the value of the stat in the results dict
                        results[delay][bank_util][seq_bytes][state] = senergy

    # To add last traffic gen idle period stats to the results dict
    for line in stats_file:
        if "system.mem_ctrls_0.memoryStateTime" in line:
            line = line.strip()  # remove leading and trailing white spaces
            # Example format:
            # 'system.mem_ctrls_0.memoryStateTime::ACT    1000000'
            statistic, stime = line.split()[0:2]
            # Now grab the state energy, .e.g 'ACT'
            state = statistic.split("::")[1]
            idleResults[state] = int(stime)
            if state == "ACT_PDN":
                break

    ########################################
    # Call plot functions
    ########################################
    # one plot per delay value
    for delay in delayValues:
        plot_path = plot_dir + delay + "-"

        plotStackedStates(
            delay,
            States,
            "IDLE",
            stateTimePlotName(plot_path),
            "Time (ps) spent in a power state",
        )
        plotStackedStates(
            delay,
            EnergyStates,
            "ACT_E",
            stateEnergyPlotName(plot_path),
            "Energy (pJ) of a power state",
        )
    plotIdle(plot_dir)


def plotIdle(plot_dir):
    """
    Create a bar chart for the time spent in power states during the idle phase

    @param plot_dir: the dir to output the plots
    """
    fig, ax = plt.subplots()
    width = 0.35
    ind = np.arange(len(States))
    l1 = ax.bar(ind, [idleResults[x] for x in States], width)

    ax.xaxis.set_ticks(ind + width / 2)
    ax.xaxis.set_ticklabels(States)
    ax.set_ylabel("Time (ps) spent in a power state")
    fig.suptitle("Idle 50 us")

    print("saving plot:", idlePlotName(plot_dir))
    plt.savefig(idlePlotName(plot_dir), format="eps")
    plt.close(fig)


def plotStackedStates(delay, states_list, bottom_state, plot_name, ylabel_str):
    """
    Create a stacked bar chart for the list that is passed in as arg, which
    is either time spent or energy consumed in power states.

    @param delay: one plot is output per delay value
    @param states_list: list of either time or energy state names
    @param bottom_state: the bottom-most component of the stacked bar
    @param plot_name: the file name of the image to write the plot to
    @param ylabel_str: Y-axis label depending on plotting time or energy
    """
    fig, ax = plt.subplots(1, len(bankUtilValues), sharey=True)
    fig.set_figheight(stackHeight)
    fig.set_figwidth(stackWidth)
    width = barWidth
    plt.rcParams.update({"font.size": plotFontSize})

    # Get the number of seq_bytes values
    N = len(seqBytesValues)
    ind = np.arange(N)

    for sub_idx, bank_util in enumerate(bankUtilValues):

        l_states = {}
        p_states = {}

        # Must have a bottom of the stack first
        state = bottom_state

        l_states[state] = [
            results[delay][bank_util][x][state] for x in seqBytesValues
        ]
        p_states[state] = ax[sub_idx].bar(
            ind, l_states[state], width, color=StackColors[state]
        )

        time_sum = l_states[state]
        for state in states_list[1:]:
            l_states[state] = [
                results[delay][bank_util][x][state] for x in seqBytesValues
            ]
            # Now add on top of the bottom = sum of values up until now
            p_states[state] = ax[sub_idx].bar(
                ind,
                l_states[state],
                width,
                color=StackColors[state],
                bottom=time_sum,
            )
            # Now add the bit of the stack that we just ploted to the bottom
            # resulting in a new bottom for the next iteration
            time_sum = [
                prev_sum + new_s
                for prev_sum, new_s in zip(time_sum, l_states[state])
            ]

        ax[sub_idx].set_title(f"Bank util {bank_util}")
        ax[sub_idx].xaxis.set_ticks(ind + width / 2.0)
        ax[sub_idx].xaxis.set_ticklabels(seqBytesValues, rotation=45)
        ax[sub_idx].set_xlabel("Seq. bytes")
        if bank_util == bankUtilValues[0]:
            ax[sub_idx].set_ylabel(ylabel_str)

    myFontSize = "small"
    fontP = FontProperties()
    fontP.set_size(myFontSize)
    fig.legend([p_states[x] for x in states_list], states_list, prop=fontP)

    plt.savefig(plot_name, format="eps", bbox_inches="tight")
    print("saving plot:", plot_name)
    plt.close(fig)


# These plat name functions are also called in the main script
def idlePlotName(plot_dir):
    return plot_dir + "idle.eps"


def stateTimePlotName(plot_dir):
    return plot_dir + "state-time.eps"


def stateEnergyPlotName(plot_dir):
    return plot_dir + "state-energy.eps"


def initResults():
    for delay in delayValues:
        results[delay] = {}
        for bank_util in bankUtilValues:
            results[delay][bank_util] = {}
            for seq_bytes in seqBytesValues:
                results[delay][bank_util][seq_bytes] = {}
