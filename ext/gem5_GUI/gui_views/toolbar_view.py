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

import sys
import random
import copy
import json
import logging

from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *

from dialogs import *
from graphic_scene import *
from gui_views import state
from m5_calls import *

class ToolBarView():
    def __init__(self, layout, state, window):
        """set up the UI and connect button to its function"""
        self.state = state

        # Create toolbar and add a button to it
        self.toolbar = window.addToolBar("tools")
        self.drawWire = QAction(QIcon("images/draw_wire.png"), "draw wire",
                                window)
        self.drawWire.setShortcut("Ctrl+W")
        self.drawWire.triggered.connect(self.wireButtonPressed)

        # Connect the button to its associated function
        self.toolbar.addAction(self.drawWire)
        self.toolbar.setMouseTracking(True)
        self.toolbar.setCursor(QCursor(Qt.OpenHandCursor))


    def wireButtonPressed(self):
        """changes gui state to allow for wire drawing and
            disable object dragging"""
        # objects should not be movable or selectable
        self.state.dragState = not self.state.dragState
        self.state.selectState = not self.state.selectState
        self.state.drawWireState = not self.state.drawWireState
        self.state.setSymObjectFlags()

        # update cursor type immediately
        pos = QCursor.pos()
        QCursor.setPos(0, 0)
        QCursor.setPos(pos)

        # set the cursor type and button image based on wire state
        if self.state.drawWireState:
            QGuiApplication.setOverrideCursor(QCursor(Qt.CrossCursor))
            self.drawWire.setIcon(QIcon("images/wire_pressed.png"))
        else:
            QGuiApplication.restoreOverrideCursor()
            self.drawWire.setIcon(QIcon("images/draw_wire.png"))
            self.state.lineDrawer.pos1 = None

        # update connections
        self.state.lineDrawer.update()
