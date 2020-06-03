# Copyright (c) 2020 Jason Lowe-Power
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
#

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from gui_views.dialogs import *

"""This class holds the visual representation of a wire connecting two ports.
It is composed of a line and an arrow head made up of two lines. This class
handles inspection and deletion of wires."""

class Wire(QGraphicsItemGroup):
    def __init__(self, line, pen, state):
        super(Wire, self).__init__()
        # set line passed in
        graphicsLine = QGraphicsLineItem(line)
        graphicsLine.setPen(pen)
        self.parentKey = None
        self.childKey = None
        self.state = state

        self.addToGroup(graphicsLine)

        # create both lines for arrowhead
        self.createArrowHead(line, 135, pen)
        self.createArrowHead(line, -135, pen)

    def createArrowHead(self, line, angle, pen):
        """Create one of the two lines composing the arrowhead"""
        # get the endpoints of the line
        x = line.x2()
        y = line.y2()

        # arrow line construction
        arrowhead = QLineF(x, y, x + 5, y + 5)
        arrowhead.setAngle(line.angle() + angle)
        arrow = QGraphicsLineItem(arrowhead)
        arrow.setPen(pen)
        self.addToGroup(arrow)


    def contextMenuEvent(self, event):
        menu = QMenu()
        delete_action = menu.addAction("delete wire")
        inspect_action = menu.addAction("inspect wire")
        selected_action = menu.exec_(QCursor.pos())
        if selected_action == delete_action:
            self.deleteWire()
        elif selected_action == inspect_action:
            pass

    def deleteWire(self):
        """delete all backend entries associate with connection and remove
        from scene"""
        parent = self.state.symObjects[self.childKey[1]]
        child = self.state.symObjects[self.parentKey[1]]

        # confirm deletion
        dialog = deleteWireDialog("Delete connection between " + parent.name +
                                "." + self.parentKey[2] + " and " + child.name
                                + "." + self.parentKey[3] + "?")
        # if yes, delete
        if dialog.exec_():
            # reset port values to default
            parent.instancePorts[self.parentKey[2]]['Value'] = \
                    parent.instancePorts[self.parentKey[2]]['Default']
            child.instancePorts[self.childKey[2]]['Value'] = \
                    child.instancePorts[self.childKey[2]]['Default']
            #delete connection from each object's connection dictionary
            del parent.uiConnections[self.parentKey]
            del child.uiConnections[self.childKey]
            #remove item from scene, add action to history
            self.state.scene.removeItem(self)
            self.state.addToHistory()

    # TODO: fix and finish this function
    def inspect(self):
        """ Create inspect popup that lists out details for a specific wire """
        pass
        # parent = self.state.symObjects[self.childKey[1]]
        # child = self.state.symObjects[self.parentKey[1]]
        # parent_port = self.parentKey[2]
        # child_port = self.parentKey[3]
