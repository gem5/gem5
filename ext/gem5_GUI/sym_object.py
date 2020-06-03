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

import copy
import sys

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from gui_views import state
from line_drawer import *
from m5_calls import *


class SymObject(QGraphicsItemGroup):

    # Constants for resizing handles indexing
    handleTopLeft = 1
    handleTopMiddle = 2
    handleTopRight = 3
    handleMiddleLeft = 4
    handleMiddleRight = 5
    handleBottomLeft = 6
    handleBottomMiddle = 7
    handleBottomRight = 8

    handleSize = +10.0
    handleSpace = -4.0

    # Cursors for resizing handles
    handleCursors = {
        handleTopLeft: Qt.SizeFDiagCursor,
        handleTopMiddle: Qt.SizeVerCursor,
        handleTopRight: Qt.SizeBDiagCursor,
        handleMiddleLeft: Qt.SizeHorCursor,
        handleMiddleRight: Qt.SizeHorCursor,
        handleBottomLeft: Qt.SizeBDiagCursor,
        handleBottomMiddle: Qt.SizeVerCursor,
        handleBottomRight: Qt.SizeFDiagCursor,
    }


    def __init__(self, x, y, width, height, scene, component_name, name,
                    loading_from_file, state):
        super(SymObject, self).__init__()

        # common variables
        self.state = state
        self.componentName = component_name
        self.connectedObjects = []
        self.parentName = None
        self.scene = scene

        # backend members
        # instancePorts and instanceParams: keep metadata about connections
        # and are employed to make connections via m5

        # simObject and simObjectInstance: former is class, latter is class
        # instance
        self.instanceParams = {}
        self.instancePorts = {}
        self.simObject = \
            copy.deepcopy(
            self.state.instances[self.componentName])
        self.simObjectInstance = None

        # ui members
        #    rect, rectText, uiPorts, deleteButton: define the QGraphicsItem
        #    that shows up in the gui

        #    uiConnections: lineDrawer instances that allow user to draw lines
        self.x = scene.width() / 2 - width
        self.y = scene.height() / 2 - height
        self.z = 0
        self.width = width
        self.height = height
        self.name = name
        self.rect = None
        self.rectText = None
        self.deleteButton = None
        self.delete
        self.uiPorts = []
        self.uiConnections = {}

        self.handles = {}
        self.handleSelected = None
        self.mousePressPos = None
        self.mousePressRect = None
        self.setAcceptHoverEvents(True)
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.ItemSendsGeometryChanges, True)
        self.setFlag(QGraphicsItem.ItemIsFocusable, True)

        self.incomplete = False
        self.modified = 0

        # constructing the baseline ui elements
        self.initUIObject(self, 0, 0)
        # if we are loading from a file, we dont need to check for overlapping
        # and can set position. If x == -1, we are importing an object
        if loading_from_file and x != -1:
            self.x = x
            self.y = y
            self.setPos(x, y)
            self.updateHandlesPos()

        else:
            self.placeNewObject()

    def placeNewObject(self):
        """ Find the coordinates and position to place the symobject based on
            the scene """
        # set initial position to center of scene
        self.setPos(self.scene.width() / 2 - self.width,
                    self.scene.height() / 2 -self.height)

        # iterate through existing objects from left to right and check if
        # current object overlaps with any of them
        for key in self.getSortedNames():
            item = self.state.symObjects[key]
            if self.doesOverlap(item):
                self.setPos(item.scenePos().x() + item.width + 20,
                            item.scenePos().y() + item.height + 20)
                self.x = self.scenePos().x()
                self.y = self.scenePos().y()

        self.x = self.scenePos().x()
        self.y = self.scenePos().y()
        self.state.removeHighlight()
        del self.state.selectedSymObjects[:]
        self.state.selectedSymObjects.append(self)
        self.updateHandlesPos()

    def getSortedNames(self):
        """returns a list of symObject names sorted with their x coordinate"""
        sorted_names = []
        for i in range(0, len(self.state.symObjects)):
            new_object = self.getMin(sorted_names)
            if new_object:
                sorted_names.append(new_object)

        return sorted_names

    def getMin(self, sorted):
        """return the name of the next sym object"""
        min = sys.maxint
        object_name = None
        for key in self.state.symObjects:
            item = self.state.symObjects[key]
            if item.name not in sorted and item.x < min:
                min = item.x
                object_name = item.name

        return object_name

    def contextMenuEvent(self, event):
        """ Create a context menu for the symo object """
        menu = QMenu()
        copy_action = menu.addAction("copy (Ctrl+C)")
        inspect_action = menu.addAction("inspect object")
        selected_action = menu.exec_(QCursor.pos())
        if selected_action == copy_action:
            self.state.mainWindow.buttonView.copyButtonPressed()
        elif selected_action == inspect_action:
            pass

    def getParamDefaults(self):
        """Get additional info on params such as default values after
        instantiating object. This information is held in a dictionary produced
        from calling enumerate_params method on instantiated object """

        # calling enumerate_params to get exact values for parameters
        enumerate_params_dict = self.simObjectInstance.enumerateParams()

        for param, value in self.instanceParams.items():
            if(isinstance(self.instanceParams[param]["Default"], AttrProxy)):
                # want to skip proxy parameters, want to do this lazily
                continue

            if enumerate_params_dict.get(param) == None:
                # Some parameters are included in the class but not in the
                # actual parameters given in enumerateParams
                continue
            else:
                # if we load from a ui file, check if the default and value
                # params are diferent
                if self.instanceParams[param]["Value"] != \
                        self.instanceParams[param]["Default"]:
                    continue

                if enumerate_params_dict[param].default_val != "":
                    # if there is a default value
                    default = enumerate_params_dict[param].default_val
                    self.instanceParams[param]["Default"] = default
                    self.instanceParams[param]["Value"] = default
                else:
                    continue

    def refreshPortInfo(self):
        """ Get port info on simobject from m5 and update everything
            in instancePorts except for values the user inputted """
        port_dict = getPortInfo(self.simObjectInstance)
        for port, port_info in self.instancePorts.items():
            port_info["Description"] = port_dict[port]["Description"]
            port_info["Default"] = port_dict[port]["Default"]
            port_info["Type"] = port_dict[port]["Type"]
            if port_info["Value"] == None:
                port_info["Value"] = port_dict[port]["Default"]

    def refreshParamInfo(self):
        """ Get param info on simobject from m5 and update everything
            in instanceParams except for values the user inputted """
        param_dict = getParamInfo(self.simObjectInstance)

        # Some parameters are included in the class but not in the actual
        # instanceParams given in enumerateParams
        weird_params = []

        for param, param_info in self.instanceParams.items():
            if param_dict.get(param) == None:
                weird_params.append(param)
                continue

            # Check is set since some of the types for parametrs are
            # VectorParam objs
            if inspect.isclass(param_dict[param]["Type"]):
                self.instanceParams[param]["Type"] = param_dict[param]["Type"]
            else:
                self.instanceParams[param]["Type"] = \
                    type(param_dict[param]["Type"])

            self.instanceParams[param]["Description"] = \
                param_dict[param]["Description"]

            if "Default" in param_dict[param]:
                self.instanceParams[param]["Default"] = \
                    param_dict[param]["Default"]
            else:
                self.instanceParams[param]["Default"] = None

            # If the value was changed in the model file then no need to load
            #   in the default, otherwise the value is set to the default
            if "Value" not in self.instanceParams[param]:
                self.instanceParams[param]["Value"] = \
                    self.instanceParams[param]["Default"]

        for i in range(len(weird_params)):
            del self.instanceParams[weird_params[i]]
        self.getParamDefaults() # use enumerateParams to assign defaults

    def initSimObject(self):
        """ Calls constructor on simobject instance """
        if self.componentName == "Root":
            self.simObjectInstance = getRoot()
        else:
            self.simObjectInstance = self.simObject()

    def instantiateSavedObj(self):
        """Instantiation and some paramter/port info collection occurs here
        when an object is loaded from a model file """
        self.initSimObject()
        self.refreshPortInfo()
        self.refreshParamInfo()

    def instantiateSimObject(self):
        """ Creates an instantiated object for the symobject and gets any new
        info on the instanceParams """
        self.initSimObject()
        self.getParamDefaults()

    def initPorts(self):
        """Create the display for the ports on the symobjects"""
        del self.uiPorts[:]
        # place the port boxes at the righmost 1/4 of the symobject
        x = self.sceneCoords().left() + \
            self.rect.boundingRect().width() * 3 / 4

        num_ports = len(self.instancePorts)
        delete_button_height = self.deleteButton.boundingRect().height()
        next_y = delete_button_height # place the ports under the delete button
        port_scale_factor = 30

        for sim_object_instance_port in sorted(self.instancePorts):
            # create dimensions for box representing p=the port
            curr_y = self.sceneCoords().top() + next_y
            port_box = QGraphicsRectItem(x, curr_y, \
                self.rect.boundingRect().width() / 4, \
                (self.rect.boundingRect().height() - \
                delete_button_height) / num_ports)
            self.addToGroup(port_box)

            # Add text and font within the box
            port_name = QGraphicsTextItem(sim_object_instance_port)
            font = QFont()
            font.setPointSize(self.rect.boundingRect().width() / \
                port_scale_factor)
            port_name.setFont(font)
            port_name.setPos(port_box.boundingRect().center() - \
                port_name.boundingRect().center())
            self.addToGroup(port_name)

            # store pointers to these port graphics items
            self.uiPorts.append((sim_object_instance_port, \
                port_box, port_name))

            # get next y position of the nect port
            height =  self.rect.boundingRect().height()
            next_y += (height - delete_button_height) / num_ports

    def movePorts(self):
        """ Used to refresh the port boxes on the sym objecy graphic """
        for port in self.uiPorts:
            port_box = port[1]
            port_name = port[2]
            self.removeFromGroup(port_box)
            self.state.scene.removeItem(port_box)
            self.removeFromGroup(port_name)
            self.state.scene.removeItem(port_name)
        self.initPorts() # recreate the port boxes

    def initUIObject(self, object, x, y):
        """creates the QGraphicsItem that shows up in the scene"""

        # initializing to (x, y) so that future positions are relative to (x,y)
        object.rect = QGraphicsRectItem(x, y, object.width, object.height)
        object.rect.setBrush(QColor("White"))

        # textbox to display symObject name
        display_name = object.name + "::" + object.componentName
        object.rectText = QGraphicsTextItem(display_name)
        object.rectText.setPos(object.rect.boundingRect().topLeft())
        object.setupDynamicFonts()

        # create delete button
        object.deleteButton = QGraphicsTextItem('X')
        delete_pos = object.rect.boundingRect().topRight()\
            - object.deleteButton.boundingRect().topRight()
        object.deleteButton.setPos(delete_pos)
        object.deleteButton.hide()

        # set max width of name, 20 is the width of the delete button
        object.rectText.setTextWidth(object.width - 20)

        # add objects created above to group
        object.addToGroup(object.rect)
        object.addToGroup(object.rectText)
        object.addToGroup(object.deleteButton)

        # set flags
        object.setAcceptDrops(True)
        object.setFlag(QGraphicsItem.ItemIsMovable, True)

    def setupDynamicFonts(self):
        """resize the name text box based on the size of the object"""
        # Currently setting scaling_factor and max_font arbitrarily
        scaling_factor = 15
        max_font = 15
        font = QFont()
        size = self.rect.boundingRect().width() / scaling_factor

        # make sure name text box doesnt get too big
        if size > max_font:
            size = max_font


        font.setPointSize(size)
        self.rectText.setFont(font)

    def moveUIObject(self):
        """ Move elements of the UI object based on new poitioning """
        self.rectText.setPos(self.rect.boundingRect().topLeft())
        self.deleteButton.setPos(self.rect.boundingRect().topRight() -
                                self.deleteButton.boundingRect().topRight())
        self.rectText.setTextWidth(self.rect.boundingRect().width() - 20)
        self.setupDynamicFonts()

    def setIncomplete(self):
        """ Check if any required instance params are not set """
        incomplete = False

        for param in self.instanceParams.values():
            if param["Value"] == 'None' or param["Value"] == None:
                incomplete = True

        self.incomplete = incomplete

    def handleAt(self, point):
        """ Returns the resize handle below the given point. """
        for k, v, in self.handles.items():
            if v.contains(point):
                return k
        return None

    def hoverMoveEvent(self, moveEvent):
        """ Executed when the mouse moves over the shape (NOT PRESSED). """
        if self.isSelected() and not self.state.drawWireState:
            handle = self.handleAt(moveEvent.pos())
            if handle is None:
                cursor = Qt.OpenHandCursor # an open hand
            else:
                cursor = self.handleCursors[handle] # resize handle
            self.setCursor(cursor)

    def hoverLeaveEvent(self, moveEvent):
        """ Executed when the mouse leaves the shape (NOT PRESSED). """
        # set general GUI cursor
        if not self.state.drawWireState:
            self.setCursor(Qt.ArrowCursor)

    def mousePressEvent(self, event):
        """ Executed when the mouse presses the shape """
        # Selecting a symobject
        self.handleSelected = self.handleAt(event.pos())
        if self.handleSelected and not self.state.drawWireState:
            self.mousePressPos = event.pos()
            self.mousePressRect = self.rect.boundingRect()
            return

        # handle required operations when a sym object is clicked on
        if not self.state.drawWireState:
            self.setCursor(QCursor(Qt.ClosedHandCursor))
        self.state.objectClicked = 1
        # get object that was clicked on (since multiple objects can be stacked
        # on top of each other)
        clicked = self.getClickedObject(event)

        # bring clicked object to foreground so drag events have object clarity
        clicked.setZValue(100)

        if not clicked:
            clicked = self
        # check if mouse press is on delete button
        deletePressed = clicked.deleteButtonPressed(event)
        if deletePressed:
            clicked.delete()
            return

        clicked.attachChildren()
        super(SymObject, clicked).mousePressEvent(event)

        # add keyboard modifiers
        modifiers = QApplication.keyboardModifiers()
        if modifiers != Qt.ShiftModifier:
            # hide button on previously selected object
            self.state.removeHighlight()
            del self.state.selectedSymObjects[:]

        # add clicked to list if not present and update attributes for it
        if not clicked in self.state.selectedSymObjects:
            self.state.selectedSymObjects.append(clicked)

        if len(self.state.selectedSymObjects) == 1:
            # Show elements based on single symobject
            clicked.deleteButton.show()
            self.state.mainWindow.populateAttributes(None,
                clicked.componentName, False)
        else: # hide attribute table if clicked on multiple symobjects
            table = self.state.mainWindow.attributeView.attributeTable
            table.clear()
            table.setRowCount(0)

        # check if symobject is incomplete
        clicked.setIncomplete()
        if clicked.incomplete:
            clicked.rect.setBrush(QColor("darkRed"))
        else:
            clicked.rect.setBrush(QColor("Green"))

    def delete(self):
        """ Deletes the symobject data """
        # attach children so all children get removed when parent does
        self.attachChildren()
        # delete ui object
        self.state.scene.removeItem(self)
        self.deleteBackend()
        del self.state.selectedSymObjects[:]
        self.state.addToHistory()

    def deleteBackend(self):
        """ Remove backend datastructures of symobject """
        # delete connections from backend and ui
        for connect in self.uiConnections.keys():
            if connect[0] == "child":
                key = ("parent", str(self.name), connect[3], connect[2])
                parent_connection = self.state.symObjects[connect[1]].\
                                                            uiConnections[key]
                if parent_connection.line:
                    parent_connection.line.state.scene.removeItem(\
                                                        parent_connection.line)
                del self.state.symObjects[connect[1]].uiConnections[key]

            else:
                connection = self.uiConnections[connect]
                if connection.line:
                    connection.line.state.scene.removeItem(connection.line)

                key = ("child", str(self.name), connect[3], connect[2])
                del self.state.symObjects[connect[1]].uiConnections[key]

            del self.uiConnections[connect]

        del self.uiConnections
        # recursively delete children's backend datastructures
        for child in self.connectedObjects:
            self.state.symObjects[child].deleteBackend()
        del self.connectedObjects

        # remove the current object from it's parent connected object list
        if self.parentName:
            parent = self.state.symObjects[self.parentName]
            if self.name in parent.connectedObjects:
                parent.connectedObjects.remove(self.name)

        # delete from state
        del self.state.symObjects[self.name]

    def mouseMoveEvent(self, event):
        """ Handle changes when symobject is moved around on canvas """

        # The user is trying to manually resize the symobject rectangle
        if self.handleSelected is not None and not self.state.drawWireState:
            self.interactiveResize(event.pos(), event.scenePos())
            self.modified = 1

        # The user is moving the symobject around the scene
        if self.state.objectClicked:
            self.modifyConnections(event, self)
            self.updateChildrenConnections(event, self)
            self.state.lineDrawer.update() # update port connection lines
            super(SymObject, self).mouseMoveEvent(event)
            self.state.mostRecentSaved = False
            self.modified = 1

    def modifyConnections(self, event, sym_object):
        """ Update connection position information when an object is dragged
        around """
        # set connection to middle of port
        num_ports = len(sym_object.instancePorts)
        if not num_ports:
            return

        # get curr positions of where the ports should be placed in rectangle
        delete_button_height = sym_object.deleteButton.boundingRect().height()
        rect_height = sym_object.rect.boundingRect().height()
        rect_width = sym_object.rect.boundingRect().width()
        y_offset = (rect_height - delete_button_height) / num_ports
        new_x = sym_object.sceneCoords().left() + rect_width * 7 / 8

        for name, connection in sym_object.uiConnections.items():
            new_y = delete_button_height
            if name[0] == "parent":
                new_y += sym_object.sceneCoords().top() \
                    + connection.parentPortNum * y_offset + y_offset / 2
                new_coords = QPointF(new_x, new_y)
                key = ("child", sym_object.name, name[3], name[2])
                connection.setEndpoints(new_coords, None)
                self.state.symObjects[name[1]] \
                    .uiConnections[key] \
                    .setEndpoints(new_coords, None)
            else:
                new_y += sym_object.sceneCoords().top() \
                    + connection.childPortNum * y_offset + y_offset / 2
                new_coords = QPointF(new_x, new_y)
                key = ("parent", sym_object.name, name[3], name[2])
                connection.setEndpoints(None, new_coords)
                self.state.symObjects[name[1]] \
                    .uiConnections[key] \
                    .setEndpoints(None, new_coords)


    def updateChildrenConnections(self, event, sym_object):
        """ Update all child connections """
        for object_name in sym_object.connectedObjects:
            object = self.state.symObjects[object_name]
            self.modifyConnections(event, object)
            self.updateChildrenConnections(event, object)

    def setParentConnection(self):
        """ Based on if the object is dragged in or out of another object
            connect or discconect from the parent """

        # iterate through all sym objects on the screen and check if the
        # object's current position overlaps with any of them
        parent = self.getFrontmostOverLappingObject()
        # if an overlapping object is found -> resize, update parent name AND
        # add self to the parent's list of children
        if parent:
            self.parentName = parent.name # add new parent
            self.z = parent.z + 1 # update z index

            # update child z indices
            for child in self.connectedObjects:
                self.state.symObjects[child].z = self.z + 1

            if not self.name in parent.connectedObjects:
                parent.connectedObjects.append(self.name) # add new child
            self.resizeUIObject(parent, 1, self.width)
        else:
            if self.parentName and not \
                self.doesOverlap(self.state.symObjects[self.parentName]):
                # if the object is dragged out of a parent, remove the parent,
                # child relationship
                curParent = self.state.symObjects[self.parentName]
                curParent.connectedObjects.remove(self.name)
                self.parentName = None
                curParent.resizeParent(self)

    def mouseReleaseEvent(self, event):
        """ When mouse is release on object, update its position including the
            case where it overlaps and deal with subobject being created """
        # Change cursor
        self.state.objectClicked = 0
        if not self.state.drawWireState:
            self.setCursor(QCursor(Qt.PointingHandCursor))
        super(SymObject, self).mouseReleaseEvent(event)

        self.setParentConnection()
        for object in self.state.symObjects.values():
            object.setZValue(object.z)

        # update the object's position
        self.x = self.scenePos().x()
        self.y = self.scenePos().y()
        self.detachChildren()
        self.state.mostRecentSaved = False
        self.handleSelected = None
        self.mousePressPos = None
        self.mousePressRect = None
        if self.modified:
            self.state.addToHistory()
        self.modified = 0

    def getClickedObject(self, event):
        """ Based on mouse click position, return object w/ highest zscore """
        frontmost_object = None
        highest_zscore = -1
        for key in self.state.symObjects:
            object = self.state.symObjects[key]
            if object.isClicked(event):
                if object.z > highest_zscore:
                    highest_zscore = object.z
                    frontmost_object = object
        if (self.z > highest_zscore):
            frontmost_object = self
        return frontmost_object

    def getFrontmostOverLappingObject(self):
        """ Based on object being dragged, return object w/ highest zscore """
        frontmost_object = None
        highest_zscore = -1
        for key in self.state.symObjects:
            object = self.state.symObjects[key]
            # if two objects are related
            if self != object and not self.isAncestor(object) and not\
                self.isDescendant(object):
                if self.doesOverlap(object):
                    if object.parentName != self.parentName \
                        or self.parentName is None:
                        if object.z > highest_zscore:
                            highest_zscore = object.z
                            frontmost_object = object

        return frontmost_object

    def isClicked(self, event):
        """ Determine if a symobject was clicked on given the position of the
        click """
        click_x, click_y = event.scenePos().x(), event.scenePos().y()
        # return if the click position is within the text item's bounding box
        if (click_x > self.scenePos().x()
            and click_x < self.scenePos().x() + self.width \
            and click_y > self.scenePos().y() \
            and click_y < self.scenePos().y() + self.height):
            return True

        return False

    def isAncestor(self, item):
        """ Determine if item is an ancestor of self """
        if not self.parentName:
            return False
        current_item = self
        while current_item.parentName:
            if current_item.parentName == item.name:
                return True
            current_item = self.state.symObjects[current_item.parentName]
        return False

    def isDescendant(self, item):
        """ Determine if item is a descendant of self """
        if item.name in self.connectedObjects:
            return True
        for child_name in self.connectedObjects:
            if self.state.symObjects[child_name].isDescendant(item):
                return True
        return False

    def deleteButtonPressed(self, event):
        """checks if the delete button was pressed based on mouse click"""

        # get x and y coordinate of mouse click
        click_x, click_y = event.scenePos().x(), event.scenePos().y()

        # get coordinate and dimension info from delete_button
        delete_button_x = self.deleteButton.scenePos().x()
        delete_button_y = self.deleteButton.scenePos().y()
        delete_button_width = self.deleteButton.boundingRect().size().width()
        delete_button_height = self.deleteButton.boundingRect().size().height()

        # if the click position is within the text item's bounding box, return
        # true
        if (click_x > delete_button_x \
            and click_x < delete_button_x + delete_button_width \
            and click_y > delete_button_y \
            and click_y < delete_button_y + delete_button_width) \
            and self.deleteButton.isVisible():
            return True

        return False

    def doesOverlap(self, item):
        """ Checks if two objects overlap """
        l1_x = self.sceneCoords().left()
        l1_y = self.sceneCoords().top()
        r1_x = self.sceneCoords().right()
        r1_y = self.sceneCoords().bottom()
        l2_x = item.sceneCoords().left()
        l2_y = item.sceneCoords().top()
        r2_x = item.sceneCoords().right()
        r2_y = item.sceneCoords().bottom()
        notoverlap = l1_x > r2_x or l2_x > r1_x or l1_y > r2_y or l2_y > r1_y
        return not notoverlap

    def sceneCoords(self):
        """ Converts the sym_object rectangle coords into scene coords """
        return self.mapToScene(self.boundingRect()).boundingRect()

    def resizeUIObject(self, item, force_resize, size):
        """ Resizes a sym_object when another object is placed in it """

        # check if resize is needed
        ret = self.resizeNeeded(item)
        if ret == 0:
            return # no need to resize object
        elif ret == -1 and size > 0: # shrink object
            size = -size

        # Update dimensions after resizing
        new_rect = item.rect.rect()
        item.width += size
        new_rect.setWidth(new_rect.width() + size)
        item.height += size / 2
        new_rect.setHeight(new_rect.height() + size / 2)
        item.rect.setRect(new_rect)
        item.updateHandlesPos()

        if ret == 1: # set postion of object
            self.setPos(item.scenePos().x(),
                item.scenePos().y() \
                + new_rect.height() \
                - self.rect.rect().height())
            self.x = self.scenePos().x()
            self.y = self.scenePos().y()

        # recursively traverse upwards and resize each parent
        if item.parentName:
            item.resizeUIObject(self.state.symObjects[item.parentName], \
            1, size)

        # Move the UI elements of both resized objects
        self.moveUIObject()
        self.movePorts()
        item.moveUIObject()
        item.movePorts()

        self.modifyConnections(item, item)
        self.updateChildrenConnections(item, item)
        self.state.lineDrawer.update()

    def resizeNeeded(self, item):
        """ Determines if a resize is needed by calculating ratio of children
        area to parent area """
        parent_area = item.rect.rect().width() * item.rect.rect().height()
        child_area = 0
        resize_threshold = .4
        for object_name in item.connectedObjects:
            object_rect = self.state.symObjects[object_name].rect.rect()
            child_area += object_rect.width() * object_rect.height()

        ratio = child_area / parent_area
        if ratio < resize_threshold / 2:
            return -1 # shrink
        elif ratio > resize_threshold:
            return 1 # grow
        else:
            return 0 # stay the same


    def lowestChild(self, item):
        """ Finds a parent's lowest child """
        lowest = item
        y_coord = item.scenePos().y() + item.height
        for child in self.connectedObjects:
            cur_child = self.state.symObjects[child]
            if (cur_child.scenePos().y() + cur_child.height > y_coord):
                y_coord = cur_child.scenePos().y() + cur_child.height
                lowest = cur_child
        return lowest

    def rightMostChild(self, item):
        """ Finds a parent's rightmost child """
        rightmost = item
        x_coord = item.scenePos().x() + item.width
        for child in self.connectedObjects:
            cur_child = self.state.symObjects[child]
            if (cur_child.scenePos().x() + cur_child.width > x_coord):
                x_coord = cur_child.scenePos().x() + cur_child.width
                rightmost = cur_child
        return rightmost

    def attachChildren(self):
        """ Attaches all children of the current sym_object to
        it so they move as one """
        for child_name in self.connectedObjects:
            self.addToGroup(self.state.symObjects[child_name])
            # attach descendants
            self.state.symObjects[child_name].attachChildren()

    def detachChildren(self):
        """ Detaches children to allow for independent movement """
        for child_name in self.connectedObjects:
            self.removeFromGroup(self.state.symObjects[child_name])
            self.state.symObjects[child_name].detachChildren()

    def updateName(self, newName):
        """ Updates a symobjects name """

        # changed name on visualization of symobject
        self.rectText.setPlainText(newName + "::" + self.componentName)

        # if sym object has a parent, change current sym object's name in
        # parent's list of child objects
        if self.parentName:
            self.state.symObjects[self.parentName].connectedObjects.\
                                                            remove(self.name)
            self.state.symObjects[self.parentName].connectedObjects.\
                                                            append(newName)

        # if sym object is a parent, change the parent name of all of its
        # children
        if self.connectedObjects:
            for child_name in self.connectedObjects:
                self.state.symObjects[child_name].parentName = newName

        # update the current object's name in each of its connections'
        # connection list
        for ui_connection in self.uiConnections:
            connected_object_name = ui_connection[1]
            connected_object = self.state.symObjects[connected_object_name]
            for connection in connected_object.uiConnections:
                if connection[1] == self.name:
                    value = connected_object.uiConnections[connection]
                    del connected_object.uiConnections[connection]
                    new_key = (connection[0], newName, connection[2],
                                connection[3])
                    connected_object.uiConnections[new_key] = value

        # update member variable
        del self.state.symObjects[self.name]
        self.name = newName
        self.state.symObjects[newName] = self
        self.state.addToHistory()

    def addSubObject(self, child):
        """ Add child to parent's (self's) UI object and setting parameters """
        child.parentName = self.name
        child.z = self.z + 1
        if not child.name in self.connectedObjects:
            self.connectedObjects.append(child.name)
        child.resizeUIObject(self, 1, child.width)
        for object in self.state.symObjects.values():
            object.setZValue(object.z)

    def resizeParent(self, child):
        """ Reduce the size of the parent if it had one child """
        child.resizeUIObject(self, 1, child.width)
        self.moveUIObject()
        self.movePorts()
        self.modifyConnections(self, self)
        self.updateChildrenConnections(self, self)
        self.state.lineDrawer.update()

    def removeUIObjects(self):
        """ Disconnect all the symobjects components before it is resized """

        self.removeFromGroup(self.rect)
        self.state.scene.removeItem(self.rect)
        self.removeFromGroup(self.rectText)
        self.state.scene.removeItem(self.rectText)
        self.removeFromGroup(self.deleteButton)
        self.state.scene.removeItem(self.deleteButton)

        for port in self.uiPorts:
            port_box = port[1]
            port_name = port[2]
            self.removeFromGroup(port_box)
            self.state.scene.removeItem(port_box)
            self.removeFromGroup(port_name)
            self.state.scene.removeItem(port_name)

    def boundingRect(self):
        """ Returns the bounding rect of the shape (including the resize
            handles). """
        o = self.handleSize + self.handleSpace
        return self.rect.rect().adjusted(-o, -o, o, o)

    def updateHandlesPos(self):
        """ Update current resize handles according to the shape size and
            position. """
        s = self.handleSize
        b = self.boundingRect()
        self.handles[self.handleTopLeft] = \
            QRectF(b.left(), b.top(), s, s)
        self.handles[self.handleTopMiddle] = \
            QRectF(b.center().x() - s / 2, b.top(), s, s)
        self.handles[self.handleTopRight] = \
            QRectF(b.right() - s, b.top(), s, s)
        self.handles[self.handleMiddleLeft] = \
            QRectF(b.left(), b.center().y() - s / 2, s, s)
        self.handles[self.handleMiddleRight] = \
            QRectF(b.right() - s, b.center().y() - s / 2, s, s)
        self.handles[self.handleBottomLeft] = \
            QRectF(b.left(), b.bottom() - s, s, s)
        self.handles[self.handleBottomMiddle] = \
            QRectF(b.center().x() - s / 2, b.bottom() - s, s, s)
        self.handles[self.handleBottomRight] = \
            QRectF(b.right() - s, b.bottom() - s, s, s)

        self.width = self.rect.rect().width()
        self.height = self.rect.rect().height()
        self.x = self.sceneCoords().left()
        self.y = self.sceneCoords().bottom()
        self.state.lineDrawer.update()

    def moveTopLeft(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging top left corner """
        fromX = self.mousePressRect.left()
        fromY = self.mousePressRect.top()
        toX = fromX + mousePos.x() - self.mousePressPos.x()
        toY = fromY + mousePos.y() - self.mousePressPos.y()
        diff.setX(toX - fromX)
        diff.setY(toY - fromY)
        boundingRect.setLeft(toX)
        boundingRect.setTop(toY)
        rect.setLeft(boundingRect.left() + offset)
        rect.setTop(boundingRect.top() + offset)
        self.rect.setRect(rect)

    def moveTopMiddle(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging top middle of the symobject """
        fromY = self.mousePressRect.top()
        toY = fromY + mousePos.y() - self.mousePressPos.y()
        diff.setY(toY - fromY)
        boundingRect.setTop(toY)
        rect.setTop(boundingRect.top() + offset)
        self.rect.setRect(rect)

    def moveTopRight(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging top right corner of the symobject """
        fromX = self.mousePressRect.right()
        fromY = self.mousePressRect.top()
        toX = fromX + mousePos.x() - self.mousePressPos.x()
        toY = fromY + mousePos.y() - self.mousePressPos.y()
        diff.setX(toX - fromX)
        diff.setY(toY - fromY)
        boundingRect.setRight(toX)
        boundingRect.setTop(toY)
        rect.setRight(boundingRect.right() - offset)
        rect.setTop(boundingRect.top() + offset)
        self.rect.setRect(rect)

    def moveMiddleLeft(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging the middle left of the symobject """
        fromX = self.mousePressRect.left()
        toX = fromX + mousePos.x() - self.mousePressPos.x()
        diff.setX(toX - fromX)
        boundingRect.setLeft(toX)
        rect.setLeft(boundingRect.left() + offset)
        self.rect.setRect(rect)

    def moveMiddleRight(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging the middle right of the symobject """
        fromX = self.mousePressRect.right()
        toX = fromX + mousePos.x() - self.mousePressPos.x()
        diff.setX(toX - fromX)
        boundingRect.setRight(toX)
        rect.setRight(boundingRect.right() - offset)
        self.rect.setRect(rect)

    def moveBottomLeft(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging the bottom left corner of the symobject """
        fromX = self.mousePressRect.left()
        fromY = self.mousePressRect.bottom()
        toX = fromX + mousePos.x() - self.mousePressPos.x()
        toY = fromY + mousePos.y() - self.mousePressPos.y()
        diff.setX(toX - fromX)
        diff.setY(toY - fromY)
        boundingRect.setLeft(toX)
        boundingRect.setBottom(toY)
        rect.setLeft(boundingRect.left() + offset)
        rect.setBottom(boundingRect.bottom() - offset)
        self.rect.setRect(rect)

    def moveBottomMiddle(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging the bottom middle of the symobject """
        fromY = self.mousePressRect.bottom()
        toY = fromY + mousePos.y() - self.mousePressPos.y()
        diff.setY(toY - fromY)
        boundingRect.setBottom(toY)
        rect.setBottom(boundingRect.bottom() - offset)
        self.rect.setRect(rect)

    def moveBottomRight(self, mousePos, offset, boundingRect, rect, diff):
        """ Resizing by dragging the bottom right corner of the symobject """
        fromX = self.mousePressRect.right()
        fromY = self.mousePressRect.bottom()
        toX = fromX + mousePos.x() - self.mousePressPos.x()
        toY = fromY + mousePos.y() - self.mousePressPos.y()
        diff.setX(toX - fromX)
        diff.setY(toY - fromY)
        boundingRect.setRight(toX)
        boundingRect.setBottom(toY)
        rect.setRight(boundingRect.right() - offset)
        rect.setBottom(boundingRect.bottom() - offset)
        self.rect.setRect(rect)

    def interactiveResize(self, mousePos, scenePos):
        """ Perform shape interactive resize. """
        offset = self.handleSize + self.handleSpace
        boundingRect = self.boundingRect()
        # boundingRect contains the resize handles
        rect = self.rect.rect()
        diff = QPointF(0, 0)
        self.prepareGeometryChange()

        if self.handleSelected == self.handleTopLeft:
            self.moveTopLeft(mousePos, offset, boundingRect, rect, diff)
        elif self.handleSelected == self.handleTopMiddle:
            self.moveTopMiddle(mousePos, offset, boundingRect, rect, diff)
        elif self.handleSelected == self.handleTopRight:
            self.moveTopRight(mousePos, offset, boundingRect, rect, diff)
        elif self.handleSelected == self.handleMiddleLeft:
            self.moveMiddleLeft(mousePos, offset, boundingRect, rect, diff)
        elif self.handleSelected == self.handleMiddleRight:
            self.moveMiddleRight(mousePos, offset, boundingRect, rect, diff)
        elif self.handleSelected == self.handleBottomLeft:
            self.moveBottomLeft(mousePos, offset, boundingRect, rect, diff)
        elif self.handleSelected == self.handleBottomMiddle:
            self.moveBottomMiddle(mousePos, offset, boundingRect, rect, diff)
        elif self.handleSelected == self.handleBottomRight:
            self.moveBottomRight(mousePos, offset, boundingRect, rect, diff)

        self.updateHandlesPos()
        self.moveUIObject()
        self.movePorts()
        #self.state.addToHistory()

    def shape(self):
        """ Returns the shape of this item as a QPainterPath in local
            coordinates. """
        path = QPainterPath()
        path.addRect(self.rect.rect())
        if self.isSelected():
            for shape in self.handles.values():
                path.addEllipse(shape)
        return path

    def paint(self, painter, option, widget=None):
        """ Paint the node in the graphic view. """
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor(0, 0, 0, 255), 1.0, \
            Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for handle, rect in self.handles.items():
            if self.handleSelected is None or handle == self.handleSelected:
                if self in self.state.selectedSymObjects:
                    painter.drawEllipse(rect)
        self.modifyConnections(self, self)
        self.updateChildrenConnections(self, self)
