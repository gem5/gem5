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
import logging
import imp
import inspect
import json
import random
import sys
from importlib import import_module

from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import *
from graphic_scene import *

from connection import *
from dialogs import *
from gui_views import state
from m5_calls import *
from connection import *

class ButtonView(): #export, draw line, save and load self.stateuration buttons
    def __init__(self, layout, state, window):
        self.state = state

        # set up main menu - add tabs and connect each button to handler
        self.mainMenu = window.menuBar()
        self.buildMenuBar(self.mainMenu, window)


    def buildMenuBar(self, mainMenu, window):
        """build the main menu bar"""
        self.buildFileTab(mainMenu, window)
        self.buildEditTab(mainMenu, window)
        self.buildViewTab(mainMenu, window)
        self.buildRunTab(mainMenu, window)
        self.buildDebugTab(mainMenu, window)
        self.buildImportTab(mainMenu, window)


    def buildFileTab(self, mainMenu, window):
        """build the file tab"""

        newAction = QAction("New File", window)
        newAction.setShortcut("Ctrl+N")

        saveAction = QAction("Save", window)
        saveAction.setShortcut("Ctrl+S")

        saveAsAction = QAction("Save As", window)
        saveAsAction.setShortcut(QKeySequence("Ctrl+Shift+S"))

        openAction = QAction("Open", window)
        openAction.setShortcut("Ctrl+O")

        exportAction = QAction("Export UI Object", window)
        importAction = QAction("Import UI Object", window)

        #set up handlers
        newAction.triggered.connect(self.newButtonPressed)
        saveAction.triggered.connect(self.saveButtonPressed)
        saveAsAction.triggered.connect(self.saveAsUIButtonPressed)
        openAction.triggered.connect(self.openUIButtonPressed)
        exportAction.triggered.connect(self.exportObjectButtonPressed)
        importAction.triggered.connect(self.importObjectButtonPressed)

        fileMenu = mainMenu.addMenu('File')
        fileMenu.setObjectName("File")
        fileMenu.addAction(newAction)
        fileMenu.addAction(saveAction)
        fileMenu.addAction(saveAsAction)
        fileMenu.addAction(openAction)
        fileMenu.addAction(exportAction)
        fileMenu.addAction(importAction)

    def buildEditTab(self, mainMenu, window):
        """build the edit tab"""
        copyAction = QAction("Copy", window)
        copyAction.setShortcut("Ctrl+C")

        pasteAction = QAction("Paste", window)
        pasteAction.setShortcut("Ctrl+V")


        undoAction = QAction("Undo", window)
        undoAction.setShortcut("Ctrl+Z")


        redoAction = QAction("Redo", window)
        redoAction.setShortcut(QKeySequence("Ctrl+Shift+Z"))

        #handlers for the edit tab
        copyAction.triggered.connect(self.copyButtonPressed)
        pasteAction.triggered.connect(self.pasteButtonPressed)
        undoAction.triggered.connect(self.undoButtonPressed)
        redoAction.triggered.connect(self.redoButtonPressed)

        #add to menu
        editMenu = mainMenu.addMenu('Edit')
        editMenu.addAction(copyAction)
        editMenu.addAction(pasteAction)
        editMenu.addAction(undoAction)
        editMenu.addAction(redoAction)

        # store undo / redo actions to enable / disable, initially disabled
        self.undo = undoAction
        self.undo.setEnabled(False)
        self.redo = redoAction
        self.redo.setEnabled(False)

    def buildViewTab(self, mainMenu, window):
        """build the view tab"""
        zoomIn = QAction("Zoom In", window)
        zoomIn.setShortcut(QKeySequence.ZoomIn) #ctrl shift +

        zoomOut = QAction("Zoom Out", window)
        zoomOut.setShortcut(QKeySequence("Ctrl+Shift+-"))

        zoomReset = QAction("Reset Zoom", window)
        zoomReset.setShortcut(QKeySequence("Ctrl+Shift+0"))

        #actions for zoom in and zoom out
        zoomOut.triggered.connect(lambda: self.zoom(.95 * self.state.zoom))
        zoomIn.triggered.connect(lambda: self.zoom(1.05 * self.state.zoom))
        zoomReset.triggered.connect(lambda: self.zoom(1))

        viewMenu = mainMenu.addMenu('View')
        viewMenu.addAction(zoomIn)
        viewMenu.addAction(zoomOut)
        viewMenu.addAction(zoomReset)

    def buildRunTab(self, mainMenu, window):
        """build the run tab"""
        instantiateAction = QAction("Instantiate", window)
        instantiateAction.setShortcut("Ctrl+I")

        simulateAction = QAction("Simulate", window)
        simulateAction.setShortcut("Ctrl+R")

        #handlers
        instantiateAction.triggered.connect(self.exportButtonPressed)
        simulateAction.triggered.connect(self.simulateButtonPressed)

        runMenu = mainMenu.addMenu('Run')
        runMenu.addAction(instantiateAction)
        runMenu.addAction(simulateAction)
        # Grey out the actions until we can actually instantiate or simulate
        self.instantiate = instantiateAction
        self.instantiate.setEnabled(False)
        self.simulate = simulateAction
        self.simulate.setEnabled(False)

    def buildDebugTab(self, mainMenu, window):
        """build the debug tab"""
        debugAction = mainMenu.addAction('Debug')
        debugAction.setShortcut("Ctrl+D")
        debugAction.triggered.connect(self.toggleDebugWindow)

    def buildImportTab(self, mainMenu, window):
        """build the import tab"""
        importAction = mainMenu.addAction('Import')
        importAction.triggered.connect(self.importObjs)

    def updateState(self, clsmembers, name):
        tree, instances= getImportedObs(clsmembers, name)
        # update the gui catalog
        self.state.updateObjs(tree, instances, name)

    def importObjs(self):
        """Import a subclassed SimObject into the current catalog library"""
        try:
            # Open file path dialog
            full_path = QFileDialog.getOpenFileName(None, 'Open file',
                '',"python files (*.py)")[0]

            tokens = full_path.split('/')
            dir_path = '/'.join(tokens[:len(tokens) - 1]) + '/'
            sys.path.append(dir_path)

            module_name = tokens[len(tokens) - 1].split('.')[0]
            modules = [key for key in sys.modules.keys()]
            import_module(module_name, package=full_path)

            clsmembers = inspect.getmembers(sys.modules[module_name], \
                inspect.isclass)
            clsmembers = filter(lambda x: x[1].__module__ not in modules, \
                clsmembers)

            self.updateState(clsmembers, module_name)
        except ValueError:
            dialog = errorDialog(self.state, "Did not select file to import")
            logging.info("Import file not selected")
            if dialog.exec_(): return
        except:
            e = sys.exc_info()[0]
            logging.error("Importing error caused by %s" % e.__name__)


    def toggleDebugWindow(self):
        """ Event handler which proxies toggling the debug widget"""
        self.state.mainWindow.toggleDebug()

    def exportObjectButtonPressed(self):
        """export details of selected object and its children"""

        if len(self.state.selectedSymObjects) != 1:
            return

        name, ok = QInputDialog.getText(self.state.mainWindow, "Alert", \
                                        "Export object as:")
        if not ok:
            return

        # show dialog box to let user create output file
        filename = QFileDialog.getSaveFileName(None, "",
                                           "",
                                           "gem5 UI Object Files (*.obj)")[0]
        # stop if cancel is pressed
        if not filename:
            return

        # add .ui extension if filename doesn't contain it
        if ".obj" not in filename:
            filename += ".obj"

        # create list of objects to export
        subObjects = []
        object = self.state.selectedSymObjects[0]
        self.createChildList(object, subObjects)

        # using list of objects, create dictionary to convert to json
        subObjectsDict = {}
        subObjectsDict[object.name] = object

        for obj in subObjects:
            subObjectsDict[obj.name] = obj

        savedObjects = self.getOutputData(subObjectsDict)

        savedObjects["object_name"] = name
        savedObjects["parent"] = object.name
        savedObjects["parent_pos_x"] = object.scenePos().x()
        savedObjects["parent_pos_y"] = object.scenePos().y()

        # with the selected file write our JSON object
        with open(filename, 'w') as outfile:
            json.dump(savedObjects, outfile, indent=4)


    def importObjectButtonPressed(self):
        """let user select file and import object"""
        # show dialog box for user to select a file to open
        filename = QFileDialog.getOpenFileName(None, 'Open file',
       '',"gem5 UI Object Files (*.obj)")[0]

       # stop if cancel is pressed or there is an error
        if not filename:
            return

        self.importFromFile(filename)

    def importFromFile(self, filename):
        """import an object given a filename"""
        importedObjects = []

        parent_name = ""
        parent_x = ""
        parent_y = ""
        import_object_name = ""

        # read data in from the file and load each object
        with open(filename) as json_file:
            data = json.load(json_file)
            parent_name = data["parent"]
            parent_x = data["parent_pos_x"]
            parent_y = data["parent_pos_y"]
            import_object_name = data["object_name"]

            dict_z_score = 0
            new_z_score = 0

            while str(dict_z_score) not in data:
                dict_z_score += 1

            while str(dict_z_score) in data:
                cur_z_array = data[str(dict_z_score)]
                for object in cur_z_array:
                    if object["parent_name"] not in importedObjects:
                        object["x"] = -1
                        object["y"] = -1
                        object["parent_name"] = None

                    new_object = self.state.scene.loadSavedObject("component",
                                                    str(object["name"]),
                                                    object)
                    new_object.z = new_z_score
                    importedObjects.append(new_object.name)

                dict_z_score += 1
                new_z_score += 1

            # use parent's position from previous session to position each
            # child relative to parent
            parent = self.state.symObjects[parent_name]
            parent_offset_x = parent.scenePos().x() - parent_x
            parent_offset_y = parent.scenePos().y() - parent_y

            for object_name in importedObjects:
                object = self.state.symObjects[object_name]
                for connection in object.uiConnections.keys():
                    # if an object required for a connection has not been
                    # imported, delete the connection
                    if connection[1] not in importedObjects:
                        del object.uiConnections[connection]
                    else:
                        connection_obj = object.uiConnections[connection]
                        # set new position's for each connection based on
                        # parent's posiiton
                        new_parent_endpoint = \
                        QPointF(connection_obj.parentEndpoint.x() + \
                        parent_offset_x, connection_obj.parentEndpoint.y() + \
                                parent_offset_y)
                        new_child_endpoint = \
                        QPointF(connection_obj.childEndpoint.x() + \
                        parent_offset_x, connection_obj.childEndpoint.y() + \
                        parent_offset_y)
                        connection_obj.parentEndpoint = new_parent_endpoint
                        connection_obj.childEndpoint = new_child_endpoint

                # set new position's for each object based on parent's posiiton
                if object_name != parent_name:
                    x = parent.scenePos().x() + \
                        object.scenePos().x() - parent_x
                    y = parent.scenePos().y() + \
                        object.scenePos().y() - parent_y
                    object.setPos(x, y)

            self.state.lineDrawer.update()

            # add parent to list of imported Objects and add object to catalog
            if import_object_name not in self.state.importedSymObjects:
                value = {"file": filename, "parent": parent}
                self.state.importedSymObjects[import_object_name] = value
                self.state.addObjectToCatalog(parent, import_object_name)

    def createChildList(self, object, subObjects):
        """create list of children given a parent"""
        for child_name in object.connectedObjects:
            child = self.state.symObjects[child_name]
            if not child in subObjects:
                subObjects.append(child)
                self.createChildList(child, subObjects)

    def newButtonPressed(self):
        """check if any changes have been made - to save before closing"""
        if not self.state.mostRecentSaved:
            dialog = saveChangesDialog("opening a new file", self.state)
            if dialog.exec_():
                self.saveButtonPressed()

        self.clearScene()
        del self.state.history[:]
        self.state.historyIndex = 0
        self.undo.setEnabled(False)
        self.redo.setEnabled(False)

    def copyButtonPressed(self):
        """initiate copy procedure"""
        logging.debug("copy button pressed")
        if not len(self.state.selectedSymObjects):
            return

        self.state.copyState = True
        self.state.copiedObjects = list(self.state.selectedSymObjects)
        self.addChildObjects()
        #copy objects from "oldest" to "youngest"
        self.state.copiedObjects.sort(key=lambda x: x.z)

    def addChildObjects(self):
        for selectedObject in self.state.copiedObjects:
            self.addChildren(selectedObject)

    def addChildren(self, object):
        """connect objects to subobjects"""
        for child_name in object.connectedObjects:
            child = self.state.symObjects[child_name]
            if not child in self.state.copiedObjects:
                self.state.copiedObjects.append(child)
                self.addChildren(child)

    def pasteButtonPressed(self):
        """paste button procedure: paste object, then paste object
            connections"""
        if not self.state.copyState:
            return

        #keep track of all the copy names, in order to find in copyConnection
        suffixList = []
        index = 0
        for selectedObject in self.state.copiedObjects:
            suffixList.append(self.copySymObject(selectedObject))
        for selectedObject in self.state.copiedObjects:
            self.copyConnection(selectedObject, suffixList[index])
            index += 1

        self.state.copyState = False
        self.state.removeHighlight()
        del self.state.copiedObjects[:]
        self.state.lineDrawer.update()
        self.state.addToHistory()

    def copySymObject(self, selectedObject):
        """copy SymObject data"""
        object_name = selectedObject.name + "_copy"
        suffix = "_copy"
        while object_name in self.state.symObjects:
            object_name += "_copy" #add suffix iteratively until
            suffix += "_copy"

        new_object = self.state.scene.addObjectToScene("component",
                                selectedObject.componentName, object_name)
        #copy over parent - child relationship info
        if selectedObject.parentName:
            parent_name = selectedObject.parentName + suffix
            if parent_name in self.state.symObjects:
                parent = self.state.symObjects[parent_name]
                parent.addSubObject(new_object)
                for key, value in parent.instanceParams.items():
                    if value["Value"] == selectedObject.name:
                        value["Value"] = object_name
                new_object.parentName = parent_name

        #copy backend info
        new_object.instancePorts = copy.deepcopy(selectedObject.instancePorts)
        new_object.instanceParams = \
            copy.deepcopy(selectedObject.instanceParams)
        new_object.SimObject = \
            copy.deepcopy(self.state.instances[new_object.componentName])

        #calculate z value
        current_object_name = selectedObject.name
        new_object.z = 0
        while self.state.symObjects[current_object_name].parentName:
            current_object_name = \
                self.state.symObjects[current_object_name].parentName
            new_object.z += 1
        new_object.initPorts()

        new_object.instantiateSimObject()
        self.state.symObjects[object_name] = new_object

        return suffix

    def copyConnection(self, selectedObject, suffix):
        """copy connection data"""
        object_name = selectedObject.name + suffix

        new_object = self.state.symObjects[object_name]
        delete_button_height = new_object.deleteButton.boundingRect().height()
        num_ports = len(new_object.instancePorts)
        # no ports, nothing to copy
        if not num_ports:
            return
        y_offset = (new_object.height - delete_button_height) / num_ports
        new_x = new_object.scenePos().x() + new_object.width * 7 / 8
        for name, connection in selectedObject.uiConnections.items():
            if self.state.symObjects[name[1]] not in self.state.copiedObjects:
                continue
            new_y = delete_button_height

            object_name2 = self.state.symObjects[name[1]].name + suffix
            object2 = self.state.symObjects[object_name2]
            delete_button_height2 = \
                object2.deleteButton.boundingRect().height()
            num_ports2 = len(object2.instancePorts)
            y_offset2 = (object2.height - delete_button_height2) / num_ports2
            new_x2 = object2.scenePos().x() + object2.width * 7 / 8
            new_y2 = delete_button_height2
            if name[0] == "parent":
                new_y += new_object.scenePos().y() + connection.parentPortNum\
                    * y_offset + y_offset / 4
                new_y2 += object2.scenePos().y() + connection.childPortNum\
                    * y_offset2 + y_offset2 / 4
                new_coords = QPointF(new_x, new_y)
                new_coords2 = QPointF(new_x2, new_y2)

                key = ("parent", object_name2, name[2], name[3])
                new_object.uiConnections[key] = Connection(new_coords,
                new_coords2, connection.parentPortNum, connection.childPortNum)
                new_object.instancePorts[name[2]]['Value'] = \
                    str(object_name2) + "." + str(name[3])
            else:
                new_y += new_object.scenePos().y() + connection.childPortNum \
                    * y_offset + y_offset / 4
                new_y2 += object2.scenePos().y() + connection.parentPortNum \
                    * y_offset2 + y_offset2 / 4
                new_coords = QPointF(new_x, new_y)
                new_coords2 = QPointF(new_x2, new_y2)
                key = ("child", object_name2, name[2], name[3])
                new_object.uiConnections[key] = Connection(new_coords,
                new_coords2, connection.parentPortNum, connection.childPortNum)


    #TODO
    def undoButtonPressed(self):
        self.clearScene()
        self.state.historyIndex -= 1
        self.populateSceneFromHistory(\
            self.state.history[self.state.historyIndex])
        if len(self.state.selectedSymObjects):
            self.state.mainWindow.populateAttributes(None,
                self.state.selectedSymObjects[0].componentName, False)
        if not self.state.historyIndex:
            self.undo.setEnabled(False)
        self.redo.setEnabled(True)

    #TODO
    def redoButtonPressed(self):
        self.clearScene()
        self.state.historyIndex += 1
        self.populateSceneFromHistory(\
            self.state.history[self.state.historyIndex])
        if len(self.state.selectedSymObjects):
            self.state.mainWindow.populateAttributes(None,
                self.state.selectedSymObjects[0].componentName, False)
        if self.state.historyIndex == len(self.state.history) - 1:
            self.redo.setEnabled(False)
        self.undo.setEnabled(True)


    def zoom(self, val):
        """ modifies the window zoom with val"""
        self.state.zoom = val
        self.state.mainWindow.graphicsView.setTransform(QTransform().scale(val,
            val).rotate(0))
        self.state.scene.resizeScene()

    def exportButtonPressed(self):
        """creates a python file that can be run with gem5"""
        dlg = instantiateDialog(self.state)
        if dlg.exec_():
            logging.debug("Export Success!")
            self.instantiate.setEnabled(False)
            self.simulate.setEnabled(True)
            self.saveButtonPressed() #want to save before instantiation
            for object in self.state.symObjects.values():
                if object.componentName == "Root":
                    root_name , root = traverseHierarchyRoot(\
                                                self.state.symObjects, object)
                    err = instantiateModel() #actual m5 instatiation
                    #lock flow so system isnt modifiable
                    self.mainMenu.findChild(QMenu, "File").setEnabled(False)
                    self.state.dragState = False
                    self.state.setSymObjectFlags()
                    if err:
                        dialog = errorDialog(self.state, \
                            "An error occured when instantiating!")
                        if dialog.exec_(): return

    def simulateButtonPressed(self):
        """creates a python file that can be run with gem5"""
        err = simulate()
        if err:
            dialog = errorDialog(self.state,\
             "An error occured when simulating!")
            if dialog.exec_(): return

    def openUIButtonPressed(self):
        """loads .ui file into gui"""
        # check if any changes have been made - to save before closing
        if not self.state.mostRecentSaved:
            dialog = saveChangesDialog("opening a new file", self.state)
            if dialog.exec_():
                self.saveButtonPressed()

        # show dialog box for user to select a file to open
        filename = QFileDialog.getOpenFileName(None, 'Open file',
       '',"gem5 UI Files (*.ui)")[0]

       # stop if cancel is pressed or there is an error
        if not filename:
            return

        # set state filename to file that was loaded
        self.state.fileName = filename

        # get file name from path and add to window title
        tokens = filename.split('/')
        self.state.mainWindow.setWindowTitle("gem5 GUI | " + tokens[-1])

        self.clearScene()
        del self.state.history[:]
        self.state.historyIndex = 0
        self.undo.setEnabled(False)
        self.redo.setEnabled(False)

        # read data in from the file and load each object
        with open(filename) as json_file:
            data = json.load(json_file)
            self.populateScene(data)
        self.state.mostRecentSaved = True
        self.state.addToHistory()

    def clearScene(self):
        """clear graphic scene"""
        for object in self.state.symObjects.values():
            for name, connection in object.uiConnections.items():
                if connection.line:
                    self.state.scene.removeItem(connection.line)

            self.state.scene.removeItem(object)
            object.uiConnections.clear()

        self.state.symObjects.clear()

    def populateScene(self, data):
        """load object from ui file to scene"""

        # execute any code saved for user-def simobjects
        imported_modules = data['code']
        if len(imported_modules) > 1: #check if any exist
            self.loadModules(imported_modules)

        z_score = 0
        while str(z_score) in data:
            cur_z_array = data[str(z_score)]
            for object in cur_z_array:
                self.state.scene.loadSavedObject("component",
                                                object["name"], object)

            z_score += 1

        self.state.lineDrawer.update()

    def populateSceneFromHistory(self, data):
        """load objects from history into graphics scene"""
        z_score = 0
        while z_score in data:
            cur_z_array = data[z_score]
            for object in cur_z_array:
                new_object = self.state.scene.loadSavedObject("component",
                                                object["name"], object)
                for sym_object in self.state.selectedSymObjects:
                    #if selected object, need to refresh the attribute table
                    if new_object.name == sym_object.name:
                        self.state.selectedSymObjects.remove(sym_object)
                        self.state.selectedSymObjects.append(new_object)

            z_score += 1
        self.state.setSymObjectFlags()
        self.state.lineDrawer.update()


    def loadModules(self, imported_modules):
        """ User defined objects are saved with their code defs. They are
            designated by a module name, same as the file the code was importd
            from. Load the modules and code to create the object classes"""

        for module in imported_modules.keys():
            existing_modules = [key for key in sys.modules.keys()]
            if module == 'headers' or module in existing_modules:
                continue
            #create module instance in sys and get the class information
            clsmemebers = self.execCode(module, imported_modules[module], \
                imported_modules['headers'])

            # add imported code to state
            self.state.importedCode[module] = {}
            for cls in imported_modules[module].keys():
                self.state.importedCode[module][cls] = \
                imported_modules[module][cls]

            #update the gui catalog with the new objects
            self.updateState(clsmemebers, module)

    def execCode(self, module_name, imported_code, header_code):
        """ Create module for imported code saved in ui file. Then extract the
            class information for the code in the module """
        existing_modules = [key for key in sys.modules.keys()]
        new_module = imp.new_module(module_name)
        sys.modules[module_name] = new_module

        # header_code contains 'from m5.objects import *'
        # need to run this in order to call exec on the rest of code
        exec(header_code, new_module.__dict__)
        for cls in sorted(imported_code.keys()):
            exec(imported_code[cls], new_module.__dict__)

        clsmembers = inspect.getmembers(sys.modules[module_name], \
            inspect.isclass)
        # filter out classes from modules already in gem5
        clsmembers = filter(lambda x: x[1].__module__ not in existing_modules,\
            clsmembers)

        return clsmembers

    def getOutputData(self, objects):
        """build dictionary to export to file"""
        savedObjects = {}

        # iterate through the current objects on the scene and create a new
        # JSON object for each one
        for object in objects.values():
            newObject = {}
            newObject["x"] = object.scenePos().x()
            newObject["y"] = object.scenePos().y()
            newObject["z"] = object.z
            newObject["width"] = object.width
            newObject["height"] = object.height
            newObject["component_name"] = object.componentName
            newObject["name"] = object.name
            newObject["parent_name"] = object.parentName

            params = {}
            #Storing the parameters
            for param in object.instanceParams:
                params[str(param)] = {}

                # TODO: Insert err message here if a parameter has not been set
                if object.instanceParams[param]["Value"] is None:
                    logging.error("Error must set required parameter")
                    params[str(param)]["Value"] = None

                #Only need to store the values of parameters changed for now
                if object.instanceParams[param]["Default"] != \
                    object.instanceParams[param]["Value"]:
                    param_type = type(object.instanceParams[param]["Value"])
                    if (param_type == str or param_type == int or \
                            param_type == bool or param_type == unicode or \
                            param_type == list):
                        params[str(param)]["Value"] = \
                            object.instanceParams[param]["Value"]
                    else:
                        # weird case if a value is a class but shouldn't rlly
                        #   hit this case since all user inputs are strings
                        params[str(param)]["Value"] = \
                            object.instanceParams[param]["Value"].__dict__

            newObject["parameters"] = params

            ports = {}
            for port in object.instancePorts.keys():
                ports[port] = {}
                if isinstance(object.instancePorts[port]["Value"], str):
                    ports[port]["Value"] = \
                        str(object.instancePorts[port]["Value"])
                else:
                    ports[port]["Value"] = None

            newObject["ports"] = ports
            newObject["connected_objects"] = copy.copy(object.connectedObjects)

            connections = []

            #storing connection metadata
            for c in object.uiConnections:
                newConnection = {}
                newConnection["key"] = c
                newConnection["parent_endpoint_x"] = \
                        object.uiConnections[c].parentEndpoint.x()
                newConnection["parent_endpoint_y"] = \
                        object.uiConnections[c].parentEndpoint.y()
                newConnection["child_endpoint_x"] = \
                        object.uiConnections[c].childEndpoint.x()
                newConnection["child_endpoint_y"] = \
                        object.uiConnections[c].childEndpoint.y()
                newConnection["parent_port_num"] = \
                        object.uiConnections[c].parentPortNum
                newConnection["child_port_num"] = \
                        object.uiConnections[c].childPortNum
                connections.append(newConnection)

            newObject["connections"] = connections

            if object.z not in savedObjects:
                savedObjects[object.z] = []

            savedObjects[object.z].append(newObject)
            savedObjects['code'] = self.state.importedCode

        return savedObjects

    def saveButtonPressed(self):
        """saves current state to open file if it exists, otherwise use dialog
        to select new file to save to"""
        # check if file is already open
        if self.state.fileName:
            filename = self.state.fileName
        else:             # show dialog box to let user create output file
            filename = QFileDialog.getSaveFileName(None, "",
                                           "",
                                           "gem5 UI Files (*.ui)")[0]

        # stop if cancel is pressed
        if not filename:
            return

        # add .ui extension if filename doesn't contain it
        if ".ui" not in filename:
            filename += ".ui"

        savedObjects = self.getOutputData(self.state.symObjects)

        # with the selected file write our JSON object
        with open(filename, 'w') as outfile:
            json.dump(savedObjects, outfile, indent=4)
        # get file name from path
        tokens = filename.split('/')
        self.state.mainWindow.setWindowTitle("gem5 GUI | " + tokens[-1])

        self.state.mostRecentSaved = True

    def saveAsUIButtonPressed(self):
        """saves gui state to a .ui file, shows dialog to select output file
        regardless of whether file exists in the state"""
        # show dialog box to let user create output file
        filename = QFileDialog.getSaveFileName(None, "",
                                           "",
                                           "gem5 UI Files (*.ui)")[0]
        # stop if cancel is pressed
        if not filename:
            return

        # add .ui extension if filename doesn't contain it
        if ".ui" not in filename:
            filename += ".ui"

        self.state.fileName = filename

        savedObjects = self.getOutputData(self.state.symObjects)

        # with the selected file write our JSON object
        with open(filename, 'w') as outfile:
            json.dump(savedObjects, outfile, indent=4)

        # get file name from path
        tokens = filename.split('/')
        self.state.mainWindow.setWindowTitle("gem5 GUI | " + tokens[-1])

        self.state.mostRecentSaved = True
