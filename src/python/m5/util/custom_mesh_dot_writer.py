# Copyright (c) 2026 Arm Limited
# All rights reserved.
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

import os

import m5
from m5.util import warn

try:
    import pydot
except:
    pydot = False


def _dot_rgb_to_html(r, g, b):
    return f"#{r:02x}{g:02x}{b:02x}"


def _strip_right(text, suffix):
    if not text.endswith(suffix):
        return text
    return text[: len(text) - len(suffix)]


def _strip_left(text, prefix):
    if not text.startswith(prefix):
        return text
    return text[len(prefix) :]


def _tooltip(path):
    return _strip_left(path, "<orphan PFSystem>.")


def _dot_create_router_node(full_path, label, pos):
    return pydot.Node(
        full_path,
        shape="box",
        label=label,
        style='"rounded, filled"',
        color="#000000",
        fillcolor=_dot_rgb_to_html(204, 230, 252),
        fontname="Arial",
        fontsize="14",
        fontcolor="#000000",
        pos=pos,
        tooltip=_tooltip(full_path),
    )


def _dot_create_rnf_router_node(full_path, label, pos):
    return pydot.Node(
        full_path,
        shape="oval",
        label=label,
        style='"filled"',
        color="#000000",
        fillcolor=_dot_rgb_to_html(204, 230, 252),
        fontname="Arial",
        fontsize="10",
        fontcolor="#000000",
        height=0,
        pos=pos,
        tooltip=_tooltip(full_path),
    )


def _dot_create_ctrl_node(full_path, label, pos):
    return pydot.Node(
        full_path,
        shape="ellipse",
        label=label,
        style='"filled"',
        color="#000000",
        fillcolor=_dot_rgb_to_html(229, 188, 208),
        fontname="Arial",
        fontsize="10",
        fontcolor="#000000",
        height=0,
        pos=pos,
        tooltip=_tooltip(full_path),
    )


def _dot_create(network, callgraph, num_rows, num_cols, all_nodes):
    # first go over the "main" mesh routers and define their position
    # and the position of all other nodes connected to them
    for r in network.routers:
        if r._main:
            pos = '"%d,%d!"' % (r._col, r._row)
            label = "R%d\n(%d,%d)" % (r.router_id, r._row, r._col)
            callgraph.add_node(_dot_create_router_node(r.path(), label, pos))

    for link in network.int_links:
        if link.src_node._main and link.dst_node._main:
            callgraph.add_edge(
                pydot.Edge(
                    link.src_node.path(),
                    link.dst_node.path(),
                    headlabel=link.latency,
                    arrowsize=0.5,
                    labeldistance=1.5,
                    fontsize=7,
                    penwidth=2,
                    tooltip=_tooltip(link.path()),
                )
            )

    if not all_nodes:
        return

    for r in network.routers:
        if not r._main:
            pos = '"%d,%d"' % (r._col, r._row)
            callgraph.add_node(
                _dot_create_rnf_router_node(r.path(), "R%d" % r.router_id, pos)
            )

    # One link for each direction but draw one edge only
    for link in network.int_links:
        if not (link.src_node._main and link.dst_node._main):
            callgraph.add_edge(
                pydot.Edge(
                    link.src_node.path(),
                    link.dst_node.path(),
                    headlabel=link.latency,
                    arrowsize=0.5,
                    labeldistance=1.5,
                    fontsize=7,
                    tooltip=_tooltip(link.path()),
                )
            )

    # Find common prefixes and sufixes to generate names
    paths = [link.ext_node.path() for link in network.ext_links]
    rpaths = [link.ext_node.path()[::-1] for link in network.ext_links]
    preffix = os.path.commonprefix(paths)
    suffix = os.path.commonprefix(rpaths)[::-1]

    for link in network.ext_links:
        ctrl = link.ext_node
        label = _strip_right(_strip_left(ctrl.path(), preffix), suffix)
        pos = '"%d,%d"' % (ctrl._col, ctrl._row)
        callgraph.add_node(_dot_create_ctrl_node(ctrl.path(), label, pos))
        callgraph.add_edge(
            pydot.Edge(
                link.ext_node.path(),
                link.int_node.path(),
                arrowhead="none",
                tooltip=_tooltip(link.path()),
            )
        )


def _generate_dot(dot_filename, network, num_rows, num_cols, all_nodes):
    callgraph = pydot.Dot(
        graph_type="digraph", overlap="scale", splines="true", sep="1.0"
    )
    _dot_create(network, callgraph, num_rows, num_cols, all_nodes)
    callgraph.write(dot_filename)
    try:
        # dot crashes if the figure is extremely wide.
        # So avoid terminating simulation unnecessarily
        callgraph.write_pdf(dot_filename + ".pdf", prog="neato")
    except:
        warn("failed to generate dot output from %s", dot_filename)


def generate_dot(network, num_rows, num_cols):
    if not pydot:
        return
    from m5 import options

    _generate_dot(
        os.path.join(options.outdir, "custom_mesh.routers.dot"),
        network,
        num_rows,
        num_cols,
        False,
    )
