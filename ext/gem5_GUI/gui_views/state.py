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
import os
import logging
import inspect

from PySide2.QtGui import *
from PySide2.QtCore import *
from PySide2.QtWidgets import *

from graphic_scene import *
from connection import *
from wire import *

class State():
    def __init__(self, instances, catalog):
        """initialize state and backend datastructures"""
        self.dragState = True # User can drag the objects
        self.selectState = True # User can select the objects
        self.drawWireState = False # User can draw wires
        self.symObjects = {} # Map name to actual symobject (has coords)
        self.selectedSymObjects = []
        self.lineDrawer = None
        self.scene = None
        self.mainWindow = None
        self.instances = instances
        self.catalog = catalog
        self.fileName = None
        self.copyState = False
        self.copiedObjects = []
        self.mostRecentSaved = True
        self.zoom = 1

        # Store imported code in state
        self.importedCode = {}
        self.importedCode['headers'] = "import m5, sys, os"
        self.importedCode['headers'] += "\nfrom m5.objects import *"
        self.importedCode['headers'] += "\nfrom common import SimpleOpts"
        self.importedSymObjects = {}
        self.objectClicked = 0
        self.historyIndex = 0
        self.history = []

    def setSymObjectFlags(self):
        """sets object flags in scene based on dragState"""
        for object in self.symObjects.values():
            object.setFlag(QGraphicsItem.ItemIsMovable, self.dragState)
            object.setFlag(QGraphicsItem.ItemIsSelectable, self.selectState)
            object.setFlag(QGraphicsItem.ItemIsFocusable, self.dragState)
            object.setAcceptHoverEvents(self.dragState)
            object.rect.setAcceptHoverEvents(self.dragState)

    def drawLines(self, p):
        """draws each line in lines using the QPen p"""
        for object in self.symObjects.values():
            for name, connection in object.uiConnections.items():
                if name[0] == "parent" and name[1] in self.symObjects:
                    self.drawConnection(p, connection, name, object.name)

    def drawConnection(self, p, connection, parent_key, parent_name):
        """draw actual line and create a wire object"""

        # remove old line if it exists
        if connection.line:
            self.scene.removeItem(connection.line)

        # instantiate a new line with connection coordinates
        line = QLineF(connection.parentEndpoint.x(), \
                connection.parentEndpoint.y(), connection.childEndpoint.x(), \
                    connection.childEndpoint.y())

        # create a new wire object so it can register mouse clicks
        wire = Wire(line, p, self)

        # add the wire to the scene
        self.scene.addItem(wire)

        connection.line = wire

        # set wire parameters that are needed for deletion
        wire.parentKey = parent_key
        wire.childKey = ("child", parent_name, parent_key[3], parent_key[2])

        connection.line.setZValue(1000)

    def removeHighlight(self):
        """clear the highlight from any selected object"""
        if len(self.selectedSymObjects):
            for sym_object in self.selectedSymObjects:
                # sets the incomplete variable of all sym objects
                sym_object.setIncomplete()
                if not sym_object.incomplete:
                    sym_object.rect.setBrush(QColor("White"))
                else:
                    sym_object.rect.setBrush(QColor("indianred"))

                sym_object.deleteButton.hide()

    def updateObjs(self, imported_catalog, imported_instances, filename):
        """Update the catalog and instance tree with new objects. """

        if filename in self.catalog:
            logging.debug("already imported file")
            return

        # Start to keep track of the imported objects
        if filename not in self.importedCode:
            self.importedCode[filename] = {}

        #Check if there are any duplicates in the imported objects
        for name in imported_instances.keys():
            if name in self.instances:
                imported_catalog[filename].pop(name, None)
                imported_instances.pop(name, None)
            elif name not in self.importedCode[filename]:
                #Store the src code in state
                src_code = inspect.getsource(imported_instances[name])
                self.importedCode[filename][name] = src_code

        self.catalog.update(imported_catalog)
        self.instances.update(imported_instances)
        self.mainWindow.repopulate(imported_catalog)

    def addObjectToCatalog(self, object, object_name):
        """add the passed in imported object to the catalog"""
        self.mainWindow.addImportedObjectToCatalog(object, object_name)

    def highlightIncomplete(self):
        """color the object red if a parameter is not set"""
        for object in self.symObjects.values():
            object.setIncomplete()
            if object.incomplete:
                object.rect.setBrush(QColor("indianred"))

    def addToHistory(self):
        state_pos = len(self.history) - self.historyIndex - 1

        # not most current state
        if state_pos > 0:
            #remove elements from history
            for i in range(state_pos):
                self.history.pop()

        history = self.mainWindow.buttonView.getOutputData(self.symObjects)
        self.history.append(history)

        self.historyIndex = len(self.history) - 1
        if self.historyIndex:
            self.mainWindow.buttonView.undo.setEnabled(True)
            self.mainWindow.buttonView.redo.setEnabled(False)
            self.mostRecentSaved = False

def get_path():
    """finds the gem5 path"""
    gem5_parent_dir = os.getenv("GEM5_HOME")

    # if parent dir not explicitly set, procure it from executable path
    if not gem5_parent_dir:
        gem5_parent_dir = sys.executable.split("gem5")[0]

    for root, dirs, files in os.walk(gem5_parent_dir, topdown=False):
        for name in dirs:
            abs_path = os.path.join(root, name)
            if abs_path.endswith("gem5/configs"):
                os.environ['gem5_path'] = abs_path
