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
import json
import random
import sys

from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *

from dialogs import *
from graphic_scene import *
from gui_views import state

class CatalogView(): #dropdown and search bar
    def __init__(self, layout, catalog, state):
        self.state = state
        self.catalog = catalog
        #search bar
        self.edit = QLineEdit()
        self.edit.setPlaceholderText("Search for an object here!")

        #adding fix width on one of the widgets in the catalog area will
        #implicitly set a fixed width on the rest of the widgets
        #we do this so that the catalog retains its original dimensions while
        #resizing the window
        self.edit.setFixedWidth(250)

        layout.addWidget(self.edit)

        #dropdown for SimObjects
        self.treeWidget = QTreeWidget()
        self.treeWidget.setObjectName("treeWidget")
        self.treeWidget.headerItem().setText(0, "Name")
        self.treeWidget.setSelectionMode(QAbstractItemView.SingleSelection)
        layout.addWidget(self.treeWidget)

        #handlers
        self.edit.textChanged.connect(self.searchItem)
        self.treeWidget.itemClicked.connect(self.state.removeHighlight)
        self.treeWidget.itemDoubleClicked.connect(self.createSymObject)

    def createSymObject(self, item):
        """this creates a new SymObject at some point in the CanvasView"""
        if item.parent() is None:
            return

        # if selecting an imported object from catalog
        if item.text(0) in self.state.importedSymObjects:
            # if the same object has already been imported, copy and paste it
            if item.text(0) in self.state.importedSymObjects and \
                    self.state.importedSymObjects\
                        [item.text(0)]["parent"].name in self.state.symObjects:

                del self.state.selectedSymObjects[:]
                self.state.selectedSymObjects.append(\
                        self.state.importedSymObjects[item.text(0)]["parent"])
                self.state.mainWindow.buttonView.copyButtonPressed()
                self.state.mainWindow.buttonView.pasteButtonPressed()
                del self.state.selectedSymObjects[:]
                return
            else:
                filename = self.state.importedSymObjects[item.text(0)]["file"]
                self.state.mainWindow.buttonView.importFromFile(filename)
                return

        name, ok = QInputDialog.getText(self.state.mainWindow, "New Object", \
                                        "New SimObject name:")
        if not ok:
            return

        if name in self.state.symObjects:
            ok = QMessageBox.about(self.state.mainWindow, "Error", \
                            "SimObject with name: " \
                            + name \
                            + " already exists!")
            if not ok:
                pass
            return

        new_parent = None

        if len(self.state.selectedSymObjects) == 1:
            new_parent = self.state.selectedSymObjects[0]

            # confirm that user wants to add a subobject to selected object
            dialog = addChildDialog("Add " + name + " as child of " + \
                                                        new_parent.name + "?")
            if not dialog.exec_():
                return

        self.state.removeHighlight()

        del self.state.selectedSymObjects[:]

        #modify state to accomodate the new object
        # print(self.catalog[item.parent().text(0)][item.text(0)]['ports'])
        new_object = \
            self.state.scene.addObjectToScene("component", item.text(0), name)
        new_object.instanceParams = \
            copy.deepcopy(self.catalog[item.parent().text(0)]\
                [item.text(0)]['params'])
        new_object.instancePorts = \
            copy.deepcopy(self.catalog[item.parent().text(0)]\
                [item.text(0)]['ports'])
        new_object.initPorts()

        #eager instantiation
        new_object.instantiateSimObject()

        #find incomplete parameters and indicate them in the AttributeView
        self.state.highlightIncomplete()

        # if sub object is being added through catalog
        if new_parent:
            child = self.state.selectedSymObjects[0]

            # need to set initial position of new object to force resize and
            # make sure there is enough space to fit object
            hasChildren = False
            if new_parent.connectedObjects:
                hasChildren = True
                lastChild = \
                    self.state.symObjects[new_parent.connectedObjects[-1]]
                child.setPos(lastChild.scenePos().x() + 10,
                    lastChild.scenePos().y() + 10)

            # configure child as a UI subobject of parent
            new_parent.addSubObject(child)

            # if parent already has child subobjects, set the new child's
            # position to the right bottom corner as it is guaranteed to be
            # empty
            if hasChildren:
                pos_x = new_parent.scenePos().x() + new_parent.width \
                                                                - child.width
                pos_y = new_parent.scenePos().y() + new_parent.height \
                                                                - child.height
                child.setPos(pos_x, pos_y)

            child.x = child.scenePos().x()
            child.y = child.scenePos().y()

            self.state.removeHighlight()
            child.rect.setBrush(QColor("Green"))
            self.state.selectedSymObjects.append(child)
            self.state.mainWindow.populateAttributes(None, child.componentName,
                                                    False)

        self.state.mostRecentSaved = False
        self.state.addToHistory()

    def searchItem(self):
        """
        Searches treeview whenever a user types something in the search bar
        """

        # Get string in the search bar and use treeview's search fn
        search_string = self.edit.text()
        match_items = self.treeWidget.findItems(search_string, \
            Qt.MatchContains | Qt.MatchRecursive)

        root = self.treeWidget.invisibleRootItem()
        child_count = root.childCount()

        # Iterate through top-level items
        for i in range(child_count):
            item = root.child(i)
            if len(match_items) == 0: # Hide all items if no matches
                item.setHidden(True)

            elif search_string == "": # if empty string don't hide or expand
                item.setHidden(False)
                item.setExpanded(False)

            else:
                # Go through sub items for each top-level item
                gchild_count = item.childCount()
                # see if any sub item is a match
                not_found = False
                for j in range(gchild_count):
                    grand_item = item.child(j)
                    not_found = not_found or (grand_item in set(match_items))
                    grand_item.setHidden(grand_item not in set(match_items))
                # hide and expand top-level item based on if sub-level item
                #   is a match
                item.setHidden(not not_found)
                item.setExpanded(not_found)
