#!/usr/bin/env python3
#
# Copyright (c) 2013 ARM Limited
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
#
# minorview.py: Minorview visuliser for MinorCPU model MinorTrace output
#

import argparse
import os
import sys

import gtk

# Find MinorView modules even if not called from minorview directory
minorviewDir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(minorviewDir)

from minorview.model import BlobModel
from minorview.point import Point
from minorview.view import (
    BlobController,
    BlobView,
    BlobWindow,
)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Minor visualiser")

    parser.add_argument(
        "--picture",
        metavar="picture-file",
        default=minorviewDir + "/minorview/minor.pic",
        help="markup file containing blob information "
        + "(default: <minorview-path>/minor.pic)",
    )
    parser.add_argument(
        "--prefix",
        metavar="name",
        default="system.cpu",
        help="name prefix in trace for CPU to be visualised (default: "
        + "system.cpu)",
    )
    parser.add_argument(
        "--start-time",
        metavar="time",
        type=int,
        default=0,
        help="time of first event to load from file",
    )
    parser.add_argument(
        "--end-time",
        metavar="time",
        type=int,
        default=None,
        help="time of last event to load from file",
    )
    parser.add_argument(
        "--mini-views",
        action="store_true",
        default=False,
        help="show tiny views of the next 10 time steps",
    )
    parser.add_argument("eventFile", metavar="event-file", default="ev")

    args = parser.parse_args(sys.argv[1:])

    model = BlobModel(unitNamePrefix=args.prefix)

    if args.picture and os.access(args.picture, os.O_RDONLY):
        model.load_picture(args.picture)
    else:
        parser.error("Can't read picture file: " + args.picture)

    # Make the key objects
    view = BlobView(model)
    controller = BlobController(
        model,
        view,
        defaultEventFile=args.eventFile,
        defaultPictureFile=args.picture,
    )
    window = BlobWindow(model, view, controller)
    window.add_control_bar(controller.bar)

    # Miniviews allow future timesteps to appear at the bottom of the
    #   display.
    if args.mini_views:
        window.miniViewCount = 10

    window.show_window()

    if args.eventFile and os.access(args.eventFile, os.O_RDONLY):
        controller.startTime = args.start_time
        controller.endTime = args.end_time
        model.load_events(
            args.eventFile, startTime=args.start_time, endTime=args.end_time
        )
        controller.set_time_index(0)
    else:
        parser.error("Can't read event file: " + args.eventFile)

    gtk.main()
