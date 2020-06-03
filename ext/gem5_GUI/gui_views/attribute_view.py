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

from graphic_scene import *
from gui_views import state


class AttributeView(): #table view for parameters, as well as the description
    def __init__(self, layout, state):
        self.state = state

        self.attributeLayout = QVBoxLayout()

        #search bar for the attributes
        self.attr_search = QLineEdit()
        self.attr_search.setPlaceholderText(\
            "Search for an attribute flag here!")
        self.attr_search.setFixedWidth(250)
        self.attr_search.textChanged.connect(self.searchAttributes)
        self.attributeLayout.addWidget(self.attr_search)

        #attribute table for an object, is editable
        self.attributeTable = QTableWidget(0,2)
        self.attributeTable.setObjectName("attributeTable")
        self.attributeTable.verticalHeader().setVisible(False)
        self.attributeTable.horizontalHeader().setVisible(False)
        header = self.attributeTable.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        self.attributeLayout.addWidget(self.attributeTable)

        layout.addLayout(self.attributeLayout)
        self.attributeTable.setMouseTracking(True)

        #description label
        self.label = QLabel()
        self.label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.label.setAlignment(Qt.AlignBottom | Qt.AlignLeft)
        self.label.setWordWrap(True)
        self.label.setScaledContents(True)
        layout.addWidget(self.label)

        #handlers
        self.attributeTable.itemDoubleClicked.connect(self.makeEditable)
        self.attributeTable.cellEntered.connect(self.changeCursor)

    def searchAttributes(self):
        """
        Searches attribute table based on search bar test
        """
        search_string = self.attr_search.text()
        for rowIndex in range(self.attributeTable.rowCount()):
            twItem = self.attributeTable.item(rowIndex, 0)
            if twItem.text().startswith(search_string):
                self.attributeTable.setRowHidden(rowIndex, False)
            else:
                self.attributeTable.setRowHidden(rowIndex, True)

    def changeCursor(self, row, col):
        if col == 1:
            self.attributeTable.setCursor(QCursor(Qt.IBeamCursor))
        else:
            self.attributeTable.setCursor(QCursor(Qt.ArrowCursor))

    def makeEditable(self, item):
        """ this function feeds into the next one, after the cell is
        changed it will trigger """
        currentColumn = self.attributeTable.column(item)
        currentRow = self.attributeTable.row(item)
        if currentColumn == 1 and \
            self.attributeTable.item(currentRow, currentColumn - 1).text() \
             == "Child Objects":
            return

        if len(self.state.selectedSymObjects) != 1 or not item or \
         self.attributeTable.item(0,0).text() != "Name" or \
         currentColumn == 0:
             return

        # set item to editable
        item.setFlags(item.flags() | Qt.ItemIsEditable)
        self.attributeTable.itemChanged.connect(self.modifyFields)


    def modifyParam(self, currentAttribute, updatedValue):
        """Given the current Attribute(param) and a new value entered in the
            gui, update the current symobject's value for the parameter"""
        instance_params = self.state.selectedSymObjects[0].instanceParams
        # if the value is name or connected objects, set the param instead of
        # the dict
        if currentAttribute not in instance_params:
            instance_params[currentAttribute] = {}
            #TODO look into this check, it seems like we do not need it
            if "Value" not in instance_params[currentAttribute]:
                catalog = self.state.catalog
                name = self.state.selectedSymObjects[0].componentName
                instance_params[currentAttribute]["Value"] = updatedValue
                instance_params[currentAttribute]["Type"] = \
                    catalog["SimObject"][name]['ports']\
                    [currentAttribute]['Type']
        else:
            instance_params[currentAttribute]["Value"] = updatedValue

        self.state.addToHistory()
        self.state.highlightIncomplete()

        # remove highlight for object if it is complete now
        object = self.state.selectedSymObjects[0]
        if not object.incomplete:
            object.rect.setBrush(QColor("Green"))


    def modifyFields(self, item):
        """ this signal disconnects itself after finishing execution,
         since we only want to trigger it AFTER a double press """

        # get attributes
        currentColumn = self.attributeTable.column(item)
        currentRow = self.attributeTable.row(item)
        try:
            currentAttribute = self.attributeTable.item(currentRow,
                                                    currentColumn - 1).text()
        except AttributeError:
            return
        currentValue = item.text()

        # if the value is name or connected objects, set the param instead of
        # the dict
        if currentAttribute == "Name":
            self.state.selectedSymObjects[0].updateName(currentValue)
            current_x = self.state.selectedSymObjects[0].x
            current_y = self.state.selectedSymObjects[0].y
            current_name = self.state.selectedSymObjects[0].name

            self.state.symObjects[current_name] = \
                self.state.selectedSymObjects[0]

        else:
            self.modifyParam(currentAttribute, currentValue)


        # item no longer editable, disconnect
        self.attributeTable.itemChanged.disconnect(self.modifyFields)
        item.setFlags(item.flags() ^ Qt.ItemIsEditable)
        if currentValue:
            item.setBackground(QColor("white"))

        self.state.mostRecentSaved = False
