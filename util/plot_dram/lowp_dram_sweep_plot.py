#! /usr/bin/python
#
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

import PlotPowerStates as plotter
import argparse
import os
from subprocess import call

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)

parser.add_argument("--statsfile", required=True, help="stats file path")

parser.add_argument(
    "--bankutils",
    default="b1 b2 b3",
    help="target bank " 'utilization values separated by space, e.g. "1 4 8"',
)

parser.add_argument(
    "--seqbytes",
    default="s1 s2 s3",
    help="no. of "
    "sequential bytes requested by each traffic gen request."
    ' e.g. "64 256 512"',
)

parser.add_argument(
    "--delays",
    default="d1 d2 d3",
    help="string of delay" ' values separated by a space. e.g. "1 20 100"',
)

parser.add_argument(
    "--outdir", help="directory to output plots", default="plot_test"
)

parser.add_argument("--pdf", action="store_true", help="output Latex and pdf")


def main():
    args = parser.parse_args()
    if not os.path.isfile(args.statsfile):
        exit(f"Error! File not found: {args.statsfile}")
    if not os.path.isdir(args.outdir):
        os.mkdir(args.outdir)

    bank_util_list = args.bankutils.strip().split()
    seqbyte_list = args.seqbytes.strip().split()
    delays = args.delays.strip().split()
    plotter.plotLowPStates(
        args.outdir + "/", args.statsfile, bank_util_list, seqbyte_list, delays
    )

    if args.pdf:
        textwidth = "0.5"

        ### Time and energy plots ###
        #############################
        # place tex and pdf files in outdir
        os.chdir(args.outdir)
        texfile_s = "stacked_lowp_sweep.tex"
        print("\t", texfile_s)
        outfile = open(texfile_s, "w")

        startDocText(outfile)
        outfile.write("\\begin{figure} \n\\centering\n")
        ## Time plots for all delay values
        for delay in delays:
            # Time
            filename = plotter.stateTimePlotName(str(delay) + "-")
            outfile.write(wrapForGraphic(filename, textwidth))
            outfile.write(getCaption(delay))
        outfile.write("\end{figure}\n")

        # Energy plots for all delay values
        outfile.write("\\begin{figure} \n\\centering\n")
        for delay in delays:
            # Energy
            filename = plotter.stateEnergyPlotName(str(delay) + "-")
            outfile.write(wrapForGraphic(filename, textwidth))
            outfile.write(getCaption(delay))
        outfile.write("\\end{figure}\n")

        endDocText(outfile)
        outfile.close()

        print("\n Generating pdf file")
        print("*******************************")
        print("\tpdflatex ", texfile_s)
        # Run pdflatex to generate to pdf
        call(["pdflatex", texfile_s])
        call(["open", texfile_s.split(".")[0] + ".pdf"])


def getCaption(delay):
    return "\\caption{" + "itt delay = " + str(delay) + "}\n"


def wrapForGraphic(filename, width="1.0"):
    # \t is tab and needs to be escaped, therefore \\textwidth
    return (
        "\\includegraphics[width=" + width + "\\textwidth]{" + filename + "}\n"
    )


def startDocText(outfile):

    start_stuff = """
\\documentclass[a4paper,landscape,twocolumn]{article}

\\usepackage{graphicx}
\\usepackage[margin=0.5cm]{geometry}
\\begin{document}
"""
    outfile.write(start_stuff)


def endDocText(outfile):

    end_stuff = """

\\end{document}

"""
    outfile.write(end_stuff)


# Call main
if __name__ == "__main__":
    main()
