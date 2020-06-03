# Gem5 Developer Guide
- Files and their purpose
- Feature Overview

## Files and their purpose

**gui.py**

Sets up the UI interface for the main window by instantiating the toolbar, catalog view, attribute view and the canvas (in the form of a graphics scene). All available gem5 simobjects are also dynamically loaded into the application and populate the catalog view. This file also contains functions that allow for the addition of objects to the catalog while importing and the population of the attribute view when an object is selected on the catalog or the canvas.

**graphics\_scene.py**

Provides the scene interface that allows for the visualization of graphical symobject items. The main functions in this class are addObjectToScene and loadSavedObject. addObjectToScene uses the name and component\_name as parameters to create a new instance of the SymObject class, which is then added to the scene and the backend state variable symObjects (which is a dictionary that keeps track of all symObjects on the canvas). Similarly, loadSavedObject is used to create symObjects from their .ui file json representations. It takes in the json object, new\_object which is then passed to the helper functions - loadSymObject, setSymObjectFields and setSymObjectConnections - to set the symObjects parameters. Similar to addObjectToScene, the new symObject is then added to the scene.

**line\_drawer.py**

The LineDrawer class is subclassed from the QWidget class to provide the line drawing functionality. To draw a line, we record the position at which the mouse was clicked as "pos1" and continuously draw a new line from pos1 to the current position of the mouse in the mouseMoveEvent (while also deleting the previously drawn line). Since there are 2 user modes - dragging an object or drawing a wire, we always check the state variable drawWireState before drawing a line. Once the mouse is released, we use the setObjectConnection function to check if the wire's endpoints (pos1 and pos2) represent a valid connection by making sure the endpoints are on actual object ports and that the identified ports are compatible. If not, we show the user an error message with the reason. If the line is valid, the setObjectConnection instantiates a Connection object which is added to both of the object's list of connections.

**Note** : to provide the line drawing functionality over the entire canvas, the lineDrawer widget is "stacked" on top of the graphics scene.

**m5\_calls.py**

This file contains a collection of functions that act as a type of Backend APIs to the gem5 repository. Most of these functions either retrieve gem5 object information or sets up the model for instantiation.

- The function getObjLists is used to load information in the GUI by returning 1) a dictionary containing a tree-like catalog which maps Base Objects to Sub Objects to the Param/Port info of said Sub Object and 2) a dictionary which maps each object name to the (un-initialized) gem5 object variable. This function makes use of two other useful functions getPortInfo and getParamInfo which take in a gem5 object and return dictionaries of the port and param information respectively.
- The function traverseHierarchyRoot takes the catalog (the same as mentioned before) and the SymObject representation of the Root object. From Root, the function goes in a DFS approach in the model representation setting the children, params, and ports of each SimObject instance.
- Since there can't be two instances of Root while running the GUI, the function getRoot serves as a Singleton type retriever for the Simobject instance of Root.
- Other functions such as instantiateModel, simulate, and getDebugFlags are just Proxy functions for the corresponding gem5 functions.

**sym\_object.py**

The symObject class is subclassed from the QGraphicsItemGroup class and represents the UI object for each simObject in the canvas. This class contains important information pertaining to both the frontend (x and y coordinates, width and height, ports) and backend (sub objects, connections, parent, parameters, component name), along with functions meant to update them. Because of the size of this class, we will be covering relevant functions in the Features Overview section below.

**connection.py**

The connection class holds information such as the parent and child port number and coordinates on the screen. The connection objects are stored in the uiConnections field of both the parent and child symObject and are used to draw and update wires.

**wire.py**

The Wire class is primarily used to register mousePressEvents on the lines by instantiating the line (previously a QLineF) as a QGraphicsLineItem. The wire class also adds an arrow head to the end of the wire to signify its direction. Wire objects also register mousePressEvents through right clicks which lets the user delete or inspect a wire. The deleteWire() function then deletes the connection object from both the parent and child's uiConnections field, after which the GraphicsItem is removed from the scene.

**attribute\_view.py**

The attributeView class sets up and defines interactions with the attribute table that displays the parameter attributes of a simObject. The makeEditable function, much like the name suggests, sets the changeable flag for the row that was double clicked. This function then feeds into the modifyField function that gets triggered if the value in the row is changed. If so, the modifyParam function is called which updates the actual simObject param dictionary with the new value. The row flag is then set to unchangeable.

**button\_view.py**

The buttonView class builds the menu bar that is displayed at the top of the window and defines the operations that are triggered when each of these buttons are pressed. As with the SymObject class, we will cover the functionality of each of these in the Features Overview.

**catalog\_view.py**

The CatalogView class sets up and defines interactions with the catalog that displays all available simObjects. The constructor builds out the widget and adds it to the mainWindow layout that's passed in. The function searchItem, which gets called everytime there is a change in the text in the search bar, defines the functionality to allow for searching within the catalog. Lastly, there is a createSymObject function which allows for the addition of symObjects when an item in the catalog is double clicked. There are 2 additional cases of symObject addition that are handled through this function:

- Adding an imported symObject cluster - when an object is imported, it gets added to the state variable importedSymObjects. If the object selected from the catalog is an importedObject, we can either re-import the object from it's file again or copy/paste the object if it exists on the canvas.
- Adding a symObject directly as a sub object of another symObject - before creating the new object, we check if any objects are currently selected. If so, we show a dialog confirming that the user wants to create a new parent-child relationship. Following this, we add the child to the parent's list of connected objects, set the parent parameter of the child and lastly position the child directly inside the parent.

**debug\_view.py**

The DebugWidget class sets up the debug window which can be displayed by pressing the debug button in the toolbar. This class contains debug logging options as well as an interface to pick gem5 debug flags. First there are two option flags for redirecting the GUI debug messages to a file, which you can rename at any time, or redirecting the messages to the terminal output. For the functionality, when you click on the checkbox for one option it unchecks the box for the other option and when the GUI is spun up redirecting to a file is the default. The class also contains a table of gem5 debug flags, retrieved from m5\_calls.py, where you can set any to be on or off. There is also an accompanying search bar for this table.

**dialogs.py**

This file provides different classes subclassed from the QDialog class for different scenarios. The basic format of each dialog is a text prompt to the user followed by an accept and reject button.

**state.py**

The State class defines variables used to keep track of the current state of the GUI and make this information available across all files. To do so, the state object is passed in to the constructor of all instantiated objects and set as a parameter. In addition to this, it provides several utility functions that can be used across all files such as setSymObjectFlags, drawLines, removeHighlight, highlightIncomplete and addToHistory, amongst others. addHistory in particular is integral to the undo functionality as it saves the current state of the GUI.

**toolbar\_view.py**

The toolbarView class is below the menu bar and contains the button to initiate the draw wire mode. When the wire button is pressed, the opposite state is set for all of the states respectively and the wire's image is updated accordingly along with the cursor type.

## Features Overview

**Drawing Wires**

Line drawing functionality is encapsulated in the lineDrawer class as described above. If the line is valid, a connection object is successfully created and stored in both the parent and child object's uiConnections field. After this, the drawLines function in the State class is called which iterates through all objects and individually redraws all the lines using their connection objects. This is done to account for any movement of symObjects on the canvas and to update accordingly. Once the QLineF object is created using information in the connection object, we create a new Wire object with the QLineF to be able to register mousePressEvents on the line. Lastly, we add the wire object to the scene.

**Saving and Loading a Configuration**

Selecting the save/save as button lets the user save the configuration in JSON format as a .ui file. The core of saving a UI configuration is encapsulated in the getOutputData function. It takes in a dictionary of symObjects as an argument and returns a JSON object which is then json dumped into the .ui file selected by the user through the File selector dialog. The getOutputData function iterates through the dictionary of symObjects and stores relevant information that would be required when the configuration is loaded.

Loading a UI file is handled through the openUIButtonPressed function. After a file is selected, the canvas is cleared to allow for the new UI configuration to be loaded. With the json data as an argument, the populateScene function iterates through the json dictionary in order of z-score and calls the loadSavedObject function to convert the json representation of the object into a symObject that is placed on the canvas. Lastly, the lineDrawer update function is called to draw the wires.

**Import SimObject**

Ideally a User should be able to create their own SimObjects for gem5 and use them in the GUI as well. For this functionality we included an import button to get these SimObjects from Python scripts. These scripts must contain class definitions for the objects to be imported. When the button is clicked the event handler function importObjs in button\_view.py is called. From there it retrieves the path from the file given by the user's choice in a pop-up window and creates a new module in sys.modules named after the file. The objects in the file that are not already in another module are then put into this module and imported. From there updateState gets the list of objects and calls getImportedObjs from m5\_calls.py to get param/port information and an instance tree (much like getObjLists). After retrieving this information, the State gets updated by adding these objects to the existing catalog and instances. It also keeps track of the code strings for each class imported.

The state keeps track of the code for each new user-defined Simobject to allow for saving and loading for these objects. In order for the User to not keep importing the same scripts everytime they load a UI file with objects that were imported, the code snippets of each object are held in state and when the model is exported they are stored in the UI file as well. When loading in the UI file, the function loadModules takes the code snippets and uses exec to run each section of code before loading the objects into state and the catalog. This has to be done before loading the actual SymObjects in.

**Export and Import a cluster of SymObjects**

Much like the load and save functionality, exporting and importing is done by saving relevant information in JSON format, this time in the form of a .obj file. To export only a select group of objects, we create a dictionary of the currently selected object (and it's children) which is then passed to getOutputData to be converted to JSON. The object data, along with some metadata about the topmost object is then saved in the .obj file.

To import back a cluster of symObjects, we read from the json file and call loadSavedObject to add it to the canvas. We also add each object to the importedObjects list to keep track of what was imported. Once all symObject have been created, we iterate through the uiConnections of each importedObject to make sure that parent-child relationships can be redrawn. If not, we delete the dictionary entry. If found, we update the position of the connection's endpoints to reflect the new position of the parent/child and this may not be the same as it was when it was exported. We then call lineDrawer's update function to draw the connections. Lastly, we add an entry to the State's importedSymObjects dictionary which will be required later on and also add the cluster to the catalog for re-use.

**Copy/Paste**

When an object is selected, clicking the copy button will copy the object and all of its children, as well as the parameters, ports, and connections between children. Paste will place the copies on the canvas, with each copied object name including "\_copy". The hierarchy and all backend information is retained, and the copied objects are instantiated.

**Undo/Redo**

Undo and redo work by saving the state of the GUI after each action that modifies the GUI, such as creating an object, deleting an object, drawing a line, modifying a parameter, etc. The state is saved similar to how a UI object is saved, by generating a json style representation of the GUI. Instead of writing to a file, we store the state in RAM in a compressed format to conserve memory. Each state is stored in an array called history, and clicking undo / redo moves through the array and switches the context between different states. Creating a new file or opening a UI file will reset the history array. There is no hard limit to the number of undo / redo actions.

**Zoom**

Zooming is done by applying a scaling transform to the canvas viewport. Each click of zoom in or zoom out will change the view by 5%. When zooming, the graphics scene, and by extension the lineDrawer canvas, needs to be manually resized as well.

**Instantiate**

Our methodology for SimObject instantiation is _semi-eager_. This means that objects and SymObject instantiation call the SimObject constructor, and are assigned the appropriate parameters for modification via dictionary storage, but all user modification is registered during the instantiation button press, where m5\_calls takes over the object processing and beings to assign children to Root, as well as the parameters that the user has modified. This means that deletion doesn't actually delete the SimObject; instead, the user can no longer reference through the GUI as the SymObject doesn't exist, which holds the SimObject reference.

Instantiate invokes the m5.instantiate() call in m5, given that each SymObject is instantiated with the appropriate SimObject. The GUI will not allow instantiation to occur until after a Root object has been placed on the canvas, at which point the option will become selectable in the toolbar.

**Simulate**

The simulate button is only enabled after the instantiate button is invoked. It invokes the m5.simulate() function.
