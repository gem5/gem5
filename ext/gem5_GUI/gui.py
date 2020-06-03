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
import functools
import logging
import sys

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

from graphic_scene import *
from gui_views.catalog_view import *
from gui_views.button_view import *
from gui_views.catalog_view import *
from gui_views.attribute_view import *
from gui_views.debug_view import *
from gui_views.toolbar_view import *
from gui_views.state import *
from m5_calls import isSimObjectParam


class MainWindow(QMainWindow):
    """this class creates the main window"""

    def __init__(self, catalog, instances):
        super(MainWindow, self).__init__()
        self.state = State(instances, catalog)
        self.main = QWidget()
        self.catalog = catalog
        self.setLayoutDirection(Qt.LeftToRight)

        self.gridLayout = QVBoxLayout()
        self.gridLayout.setObjectName("gridLayout")

        #add button view
        self.buttonView = ButtonView(self.gridLayout, self.state, self)
        #add toolbar view
        self.toolbarView = ToolBarView(self.gridLayout, self.state, self)
        #add catalog view
        self.catalogView = CatalogView(self.gridLayout, catalog, self.state)
        #add attributes
        self.attributeView = AttributeView(self.gridLayout, self.state)

        self.state.scene = GraphicsScene(0, 0, 1750, 1250, self.state)
        self.graphicsView = QGraphicsView(self.state.scene)

        self.layout = QHBoxLayout()
        self.layout.addLayout(self.gridLayout)
        self.layout.addWidget(self.graphicsView)

        #add debug window
        self.debugHidden = False # Flag for toggling debug widget
        self.debugWidget = DebugWidget(self.state)
        self.layout.addWidget(self.debugWidget)
        self.toggleDebug()

        self.main.setLayout(self.layout)
        self.setCentralWidget(self.main)

        # populate treeview
        self.populate()
        self.catalogView.treeWidget.itemClicked.connect(self.treeWidgetClicked)

    def toggleInspect(self, is_object, attribute_list):
        """ Enables or disables the inspect menu for gui items """
        if self.inspectHidden:
            self.inspectWidget.populate(is_object, attribute_list)
            self.inspectWidget.show()
        else:
            self.inspectWidget.clear()
            self.inspectWidget.hide()

    def toggleDebug(self):
        """ Enables or disables the debug widget from being shown"""
        if self.debugHidden:
            self.debugWidget.show()
        else:
            self.debugWidget.hide()
        self.debugHidden = not self.debugHidden

    def createDropDown(self, value, attr_table, param, param_type):
        """ Create the drop down for simobject parameters in the table view """
        combo_box = QComboBox()
        # Create list for dropdown including the default value
        connected_objects = copy.deepcopy(\
            self.state.selectedSymObjects[0].connectedObjects)

        dropdown_list = []
        if len(connected_objects) > 0:
            for obj in connected_objects:
                # Check if the simobject matches the type for the param
                sim_obj_type = \
                    type(self.state.symObjects[obj].simObjectInstance)
                if issubclass(sim_obj_type, param_type):
                    dropdown_list.append(obj)

        if value in dropdown_list:
            # value for the param should at the top of the drop down
            dropdown_list.remove(value)

        # Make whatever value or default value the first option
        dropdown_list = [value] + dropdown_list

        #Check if param is req
        if dropdown_list[0] == 'None':
              cbstyle = " QComboBox {"
              cbstyle += " background: red;"
              cbstyle += "}"
              combo_box.setStyleSheet(cbstyle)

        combo_box.addItems(dropdown_list)
        # Add event handler to update values in the symobject structure
        combo_box.currentTextChanged.connect(functools.partial(\
            self.attributeView.modifyParam, param))
        attr_table.setCellWidget(attr_table.rowCount() - 1, 1, combo_box)

    def parseParam(self, param):
        """parse m5.params for cleaner tooltip view"""
        new_param = param
        if param[0] == "<":
            new_param = param.lstrip("<").rstrip(">").split()[1]
        return new_param

    def addRow(self, param, value, is_tree_widget, is_sim_obj):
        """ Adds the param and value to a row of the table."""
        attr_table = self.attributeView.attributeTable
        attr_table.insertRow(attr_table.rowCount())

        # set column 0 value with param
        attr_table.setItem(attr_table.rowCount() - 1, 0, \
            QTableWidgetItem(param))
        cell_1 = attr_table.item(attr_table.rowCount() - 1, 0)
        cell_1.setFlags(cell_1.flags() ^ Qt.ItemIsEditable)

        # set column 1 value with value
        attr_table.setItem(attr_table.rowCount() - 1, 1,\
            QTableWidgetItem(value))
        if is_sim_obj: #add a drop down of child simobjects
            param_type = self.attributes[param]["Type"]
            self.createDropDown(value, attr_table, param, param_type)

        cell_2 = attr_table.item(attr_table.rowCount() - 1, 1)
        cell_2.setFlags(cell_2.flags() ^ Qt.ItemIsEditable)
        if not is_tree_widget and value == 'None': # check if param is req
            cell_2.setBackground(QColor("indianred"))

        if param != "Name" and param != "Child Objects":
            cell_1.setToolTip(self.attributes[param]["Description"])
            cell_2.setToolTip(\
                self.parseParam(str(self.attributes[param]["Type"])))

        self.state.highlightIncomplete()


    def treeWidgetClicked(self, item, name):
        self.populateAttributes(item, name, True)

    def loadAttributes(self, item, name):
        """ Set the attributes member variable, which contains values for the
        params, based on the current conext of the object selected"""
        if item:
            if item.parent() is None or item.text(0) in \
                                                self.state.importedSymObjects:
                return
            self.attributes = \
                self.catalog[item.parent().text(0)][item.text(0)]['params']
        else:
            # only load from param list if there is a sym object in the context
            if len(self.state.selectedSymObjects) == 1 or \
                self.state.selectedSymObjects[0].componentName == name:
                self.attributes = \
                    self.state.selectedSymObjects[0].instanceParams
            else: # TODO: check when would this branch happen??
                logging.debug("filling in name branch")
                self.attributes = self.catalog[name]


    def populateAttributes(self, item, name, is_tree_widget_click):
        """Populate the attribute table holding info for an objects
            params and children"""
        attr_table = self.attributeView.attributeTable
        attr_table.clear()
        attr_table.setRowCount(0)

        # If there is an object being viewed on the board display the name and
        #   connected objects as well
        if len(self.state.selectedSymObjects) == 1:
            cur_object = self.state.selectedSymObjects[0]
            self.addRow("Name", cur_object.name, is_tree_widget_click, False)
            self.addRow("Child Objects", \
                        ", ".join(cur_object.connectedObjects), \
                        is_tree_widget_click, False)

        self.loadAttributes(item, name)

        # display the param name and values
        for attribute in sorted(self.attributes.keys()):
            # Simobject params are special cases with dropdowns in the table
            isSim = False
            param = self.attributes[attribute]
            if len(self.state.selectedSymObjects) > 0:
                isSim = self.state.selectedSymObjects[0] and \
                    isSimObjectParam(param)
            self.addRow(attribute, str(param["Value"]), \
                is_tree_widget_click, isSim)

    def repopulate(self, imported_catalog):
        """Adds newly imported sub_objs and updates the catalog"""
        # Go through every inheritable sym-object
        for item in sorted(imported_catalog.keys()):
            tree_item = QTreeWidgetItem([item])
            # Go through every specialized sym-object
            for sub_item in sorted(imported_catalog[item].keys()):
                tree_item.addChild(QTreeWidgetItem([sub_item]))
            self.catalogView.treeWidget.addTopLevelItem(tree_item)
        self.catalog.update(imported_catalog)

    def populate(self):
        """ This function populates the tree view with sym-objects"""
        # Go through every inheritable sym-object
        for item in sorted(self.catalog.keys()):
            tree_item = QTreeWidgetItem([item])
            # Go through every specialized sym-object
            for sub_item in sorted(self.catalog[item].keys()):
                tree_item.addChild(QTreeWidgetItem([sub_item]))
            self.catalogView.treeWidget.addTopLevelItem(tree_item)

    def addImportedObjectToCatalog(self, object, object_name):
        """create new entry in catalog for imported objects"""
        parent_item = self.catalogView.treeWidget.findItems(\
            "Imported Objects", Qt.MatchContains)
        if not parent_item:
            self.catalog["Imported Objects"] = {}
            tree_item = QTreeWidgetItem(["Imported Objects"])
            self.catalog["Imported Objects"][object_name] = object
            tree_item.addChild(QTreeWidgetItem([object_name]))
            self.catalogView.treeWidget.addTopLevelItem(tree_item)
        else:
            parent_item[0].addChild(QTreeWidgetItem([object_name]))

    def closeEvent(self, event):
        """When user tries to exit, check if changes need to be saved
            before closing"""
        if not self.state.mostRecentSaved:
            self.dialog = saveChangesDialog("closing", self.state)
            if self.dialog.exec_():
                self.buttonView.saveButtonPressed()

if __name__ == "__m5_main__":
    import os
    import sys

    import m5.objects

    from common import ObjectList
    from m5_calls import getObjLists
    sys.path.append(os.getenv('gem5_path'))

    # use gem5 to get list of objects
    obj_tree, instance_tree = getObjLists()
    gui_application = QApplication() #create new application
    #create new instance of main window
    main_window = MainWindow(obj_tree, instance_tree)
    main_window.state.mainWindow = main_window
    main_window.state.addToHistory()
    main_window.setWindowTitle("gem5 GUI | Untitled")
    main_window.show() #make instance visible
    main_window.raise_() #raise instance to top of window stack
    gui_application.exec_() #monitor application for events
    gui_application.quit()
