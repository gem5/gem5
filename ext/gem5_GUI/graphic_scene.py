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

import collections
import copy
import random
import string

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from gui_views import state
from line_drawer import *
from sym_object import *

def convert(data):
    """convert a dictionary with unicode keys and values to utf-8"""

    if isinstance(data, basestring):
        return str(data)
    elif isinstance(data, collections.Mapping):
        return dict(map(convert, data.iteritems()))
    elif isinstance(data, collections.Iterable):
        return type(data)(map(convert, data))
    else:
        return data

class GraphicsScene(QGraphicsScene):
    """this class provides a scene to manage objects"""

    # constructor
    def __init__(self, x, y, width, height, state):
        super(GraphicsScene, self).__init__(x, y, width, height)
        self.state = state
        self.defaultWidth = width
        self.defaultHeight = height
        self.state.lineDrawer = LineDrawer(state)
        self.setLineDrawer()
        self.addWidget(self.state.lineDrawer)

    def loadSymObject(self, name, new_object):
        """Creates the sym object from the necessary UI fields"""
        x = new_object["x"]
        y = new_object["y"]
        width = new_object["width"]
        height = new_object["height"]
        component_name = convert(new_object["component_name"])
        if component_name == "Root":
            #found a root object loaded in from a ui file
            self.state.mainWindow.buttonView.instantiate.setEnabled(True)

        return SymObject(x, y, width, height, self, component_name, name,
            True, self.state)

    def setSymObjectFields(self, sym_object, new_object):
        """Sets backend fields for the sym object"""
        sym_object.instanceParams = new_object["parameters"]
        sym_object.connectedObjects = new_object["connected_objects"]
        sym_object.parentName = new_object["parent_name"]
        sym_object.z = new_object["z"]
        sym_object.instancePorts = convert(new_object["ports"])

    def setSymObjectConnections(self, sym_object, new_object):
        """Builds the connection dictionary for the sym object"""
        connections = convert(new_object["connections"])
        new_object_connections = {}

        # rebuild the connection dictionary for the object
        for connection in connections:
            parent_endpoint = QPointF(connection["parent_endpoint_x"],
                                        connection["parent_endpoint_y"])
            child_endpoint = QPointF(connection["child_endpoint_x"],
                                        connection["child_endpoint_y"])
            new_connection = Connection(parent_endpoint, child_endpoint,
                                            connection["parent_port_num"],
                                            connection["child_port_num"])
            key = (connection["key"][0], connection["key"][1],
                connection["key"][2], connection["key"][3])
            new_object_connections[key] = new_connection

        sym_object.uiConnections = new_object_connections


    def loadSavedObject(self, type, name, new_object):
        """load object from saved UI file"""
        sym_object = self.loadSymObject(name, new_object)
        self.setSymObjectFields(sym_object, new_object)
        self.setSymObjectConnections(sym_object, new_object)
        sym_object.initPorts()
        # instantiate the simobject and set its parameters
        sym_object.instantiateSavedObj()

        # add new object to backend datastructures
        self.state.symObjects[name] = sym_object

        self.state.highlightIncomplete()
        self.addItem(sym_object)
        return sym_object


    def addObjectToScene(self, type, component_name, name):
        """Creates symobject representation of object and adds to the scene"""
        # generate random string name for object if none provided by user
        if not name:
            name = ''.join(random.choice(string.ascii_lowercase)
                            for i in range(7))

        # add object rectangle to scene
        new_object = SymObject(0, 0, 150, 75, self, component_name, name,
                                False, self.state)

        self.state.symObjects[name] = new_object

        if component_name == "Root":
            #user created a root object, can instantiate now
            self.state.mainWindow.buttonView.instantiate.setEnabled(True)
        new_object.deleteButton.show()
        new_object.rect.setBrush(QColor("Green"))

        self.addItem(new_object)
        return new_object


    def resizeScene(self):
        """Resize the graphics scene based on zoom value"""
        scale = self.state.zoom
        rect = self.itemsBoundingRect()
        self.setSceneRect(rect.x(), rect.y(), self.defaultWidth / scale,
                self.defaultHeight / scale)
        self.setLineDrawer()


    def setLineDrawer(self):
        """Initialize line drawer"""
        self.state.lineDrawer.resize(self.width(), self.height())

        # change background of canvas to light gray
        pal = QPalette()
        pal.setColor(QPalette.Background, Qt.lightGray)
        self.state.lineDrawer.setAutoFillBackground(True)
        self.state.lineDrawer.setPalette(pal)
