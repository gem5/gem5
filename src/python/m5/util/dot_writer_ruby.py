# Copyright (c) 2019,2021 ARM Limited
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

# Creates a visual representation of a Ruby network topology

import os
import m5
from m5.util import warn
try:
    import pydot
except:
    pydot = False


def _dot_rgb_to_html(r, g, b):
    return "#%.2x%.2x%.2x" % (r, g, b)

def _dot_create_router_node(full_path, label):
    return pydot.Node( \
                         full_path, \
                         shape = "Mrecord", \
                         label = label, \
                         style = "\"rounded, filled\"", \
                         color = "#000000", \
                         fillcolor = _dot_rgb_to_html(204, 230, 252), \
                         fontname = "Arial", \
                         fontsize = "14", \
                         fontcolor = "#000000" \
                         )

def _dot_create_ctrl_node(full_path, label):
    return pydot.Node( \
                         full_path, \
                         shape = "Mrecord", \
                         label = label, \
                         style = "\"rounded, filled\"", \
                         color = "#000000", \
                         fillcolor = _dot_rgb_to_html(229, 188, 208), \
                         fontname = "Arial", \
                         fontsize = "14", \
                         fontcolor = "#000000" \
                         )


def _dot_create(network, callgraph):
    for r in network.routers:
        callgraph.add_node(_dot_create_router_node(r.path(),
            'R %d' % r.router_id))

    # One link for each direction but draw one edge only
    connected = dict()
    for link in network.int_links:
        if (link.src_node.path() in connected) and \
           (connected[link.src_node.path()] == link.dst_node.path()):
           continue
        callgraph.add_edge(
            pydot.Edge(link.src_node.path(), link.dst_node.path())
        )
        connected[link.dst_node.path()] = link.src_node.path()

    # Find common prefixes and sufixes to generate names
    paths = [link.ext_node.path() for link in network.ext_links]
    rpaths = [link.ext_node.path()[::-1] for link in network.ext_links]
    preffix = os.path.commonprefix(paths)
    suffix = os.path.commonprefix(rpaths)[::-1]
    def strip_right(text, suffix):
        if not text.endswith(suffix):
            return text
        return text[:len(text)-len(suffix)]
    def strip_left(text, prefix):
        if not text.startswith(prefix):
            return text
        return text[len(prefix):]


    for link in network.ext_links:
        ctrl = link.ext_node
        label = strip_right(strip_left(ctrl.path(), preffix), suffix)
        if hasattr(ctrl, '_node_type'):
            label += ' (' + ctrl._node_type + ')'
        callgraph.add_node(
            _dot_create_ctrl_node(ctrl.path(), label)
        )

        callgraph.add_edge(
            pydot.Edge(link.ext_node.path(), link.int_node.path())
        )

def _do_dot(network, outdir, dotFilename):
    callgraph = pydot.Dot(graph_type='graph', rankdir='LR')
    _dot_create(network, callgraph)
    dot_filename = os.path.join(outdir, dotFilename)
    callgraph.write(dot_filename)
    try:
        # dot crashes if the figure is extremely wide.
        # So avoid terminating simulation unnecessarily
        callgraph.write_svg(dot_filename + ".svg", prog='neato')
        callgraph.write_pdf(dot_filename + ".pdf", prog='neato')
    except:
        warn("failed to generate dot output from %s", dot_filename)


def do_ruby_dot(root, outdir, dotFilename):
    RubyNetwork = getattr(m5.objects, 'RubyNetwork', None)

    if not pydot or not RubyNetwork:
        return

    # Generate a graph for all ruby networks.
    def is_ruby_network(obj):
        return isinstance(obj, RubyNetwork)

    for network in filter(is_ruby_network, root.descendants()):
        # We assume each ruby system has a single network.
        rubydotFilename = dotFilename.replace(".dot",
                                "." + network.get_parent().path() + ".dot")
        _do_dot(network, outdir, rubydotFilename)
