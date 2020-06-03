# **gem5 GUI**


## Overview

### Background and Motivation

The current state of gem5 requires the development of python scripts to define and generate different architectures. This can be tedious to develop since the user flow can result in less visual thinking. We want a tool to have a better user flow to allow developers to create and tweak these models in a visual manner, much like Chisel. To make gem5 more accessible and allow new users to utilize all of its capabilities, we created a user interface that will allow such ease of use and functionality.


### Description

We developed a user interface that allows users to search for simobjects in the left-hand catalog, place them in the canvas, and move them around to create an architectural hierarchy that can be instantiated and simulated. Selecting an object allows a user to modify parameters in the attribute table on the bottom left. Placing objects inside other objects establishes a parent-child relationship. Drawing wires using the wire tool allows for port connections between objects.

### Approach

The image below was our initial diagramming of the basic structure of the GUI, as well as a very high-level overview of interaction with our "back-end", which in this case was the gem5 repository. Our GUI had a distinct front-end and back-end, which are linked by a State class and the SymObject class.

 ![](https://lh3.googleusercontent.com/wjVauYFnztL0aIxrHWjf-dgybE87O4_nTb2dcB3mOpZezpZfnenHZ8csDD0EOwaGaCWd_c1Ysb6HSWvdz-mbfKwMAkVXUMrjLwsyyg4A2aR-Pl3OSn_T2r-zHbBRMiNR1s6pEdnF)

### Technical Specifications

We decided to develop the GUI using just Python. So the natural choice was to use the Python binding for **QT**, one of the most popular GUI development libraries. We chose **PySide2** over PyQt5 since we would need a commercial license to release code under PyQt5.


## Installation and Setup

### 1. Prerequisites

gem5 requires the Linux operating system to run, so the GUI does not support cross-platform development. You need to have a compiled gem5 installation on your machine as well. Visit [gem5 download](http://www.m5sim.org/Download) for instructions on how to setup gem5.

### 2. Basic Setup

To begin the setup process, clone the [repository](https://github.com/afarooqui98/gem5_GUI) directly into the gem5 directory. Once complete, enter into the gem5\_GUI directory and download the dependencies using:

```pip3 install -r requirements.txt```

### 3. Running the application

Once the dependencies are installed, users can run the GUI with the command:

```<gem5.opt path> gui.py```

See the README.md file in the repository for help with setup and running issues.

## Features and Functionality

### GUI Overview

Attached below is a view of the GUI on successful launch:

**![](https://lh5.googleusercontent.com/CFVJ2WTCP-fm_eeWc1hU_a3kHUz9TNeFe7y2UpBRp8JfrFWwZfPxxWh_QshoAuESh_zDqXyp6_G2bf4PeKuDuI4COeeS1KNhSxXhTHBsy2ZYZX60d6R73K4BQ3e6vQ5xgj8yhwSs)**

On the left side lies the **catalog view** as well as the **attribute view**.

The former is used to select a SimObject, and the latter will be used to configure a selected SimObject. The majority of the screen is populated by the **canvas**. This is where most of the user interaction will occur, and where users will build their system. The menu bar contains multiple convenience functions typical to GUI software, from copy-pasting to file saving, but there are also tabs for **debugging** , **running** , and **importing**. These are key functions of the GUI that work in tandem with gem5 to provide the users with the ability to check their system configuration, import both UI objects and configured subclasses, and instantiate their systems. Underneath the top menu is a button that allows the user to draw ports between objects

### Catalog View
**![](https://lh5.googleusercontent.com/iv-iXWbl-zvDkwHlkJ9Adlp4xjj-vP9g_kb4yZYRMtSTrtOUnrlsVTdY73JieBOCWHBDno7JHm0YxuohtawUyQ5tb1EjewX45XU6Q5Z8NOC8WoIYGeZECXX4tcqR5dfbEmMt7Hp6 )**

The catalog holds all the available SimObjects. Users can maneuver the tree view by a specific category or search for an object at the top search bar. Double clicking an object places what we call a SymObject on the canvas. This is a GUI representation of an m5 SimObject that allows a user to interact with it in a tactile way.

### Attribute Table

**![](https://lh3.googleusercontent.com/gxPj5FWqqfpwkIxH--c_LYTa0eCnfYEqxaqX2iR7ZFf9UwyQQSWjLWBjjDDfbSJrE-0oWVk9rkpOMOZnRdcNzUXfyP8h1144lTWUn-Hgt96BeBitZqRTqXr8Bv8A6RCwkzb7kOdt)**

Selecting an object brings up its attribute table. This table lists the object name, every child object, as well as all the object parameters. The parameter fields are modifiable, while the "Name" and "Child Objects" fields are only viewable. Hovering over the parameter name gives a description of the parameter, hovering over the value shows the type. Attributes for the specific object may also be searched for, aside from the "Name" and "Child Objects" parameters, which always lie at the top of the table.

### Wiring

To enable wire drawing, click the wire icon between the menu and the catalog. While in wire drawing mode, objects cannot be interacted with. Clicking this button changes the cursor to a crosshair, allowing the user to connect ports with wires. Failing to connect two ports or connecting incompatible ports will result in an error message. Right-clicking a wire brings up a context menu, which will allow for deletion and inspection (printing information about the end connections).

**![](https://lh5.googleusercontent.com/k3X4PbsV-p_0oNeMGzexuSvBhwoxifQ28G0GGwRPh3QdDB7Q_zl1dCq-dSx7yF7OOA5lsIbB2maPyrQl_yaHlal2H-QIfMKeph4FpgnbwPfTdk0qWnVR9CFmdGq7VeEDV1wT5I9Q)**

### Context

An important part of understanding the gem5 GUI is the way user context works. Whenever an object is created or selected, it is set as the current selected SymObject. Objects that are selected are typically highlighted green unless there are required attributes that need to be set by the user; then, the object is red. Any time an object is selected, the user can move it around freely and resize it in the canvas, and its attributes are populated in the attribute table. Finally, it's important to note that wiring _is not_ dependent on the current context, so any ports for any objects may be connected to others regardless of whether they are in context.

### Menu Overview

The menu contains tools to interact with the GUI. Most of these correspond to self-explanatory standard window functionality, and all options in the menu correspond to a keyboard shortcut.

### Run

**![](https://lh3.googleusercontent.com/w7zPXdpEmfvKHBrRCVVazTPXdZGHD4JeemIjvkwXMbomtK5lTdlMmlwplL3d6lF66SYRkinzCPXO1FbHSE4Ou-RjZbbX17yxBO1zkqwt6NBYw23eF7eRHQUiYMHP_WxubpwfzqVR)**

The run tab contains the instantiate and simulate option. Note that it is greyed out until a user drops in at least a root object to prevent the user from instantiating without a root. Once the instantiate button is pressed, the user _must_ save the file (since objects cannot be modified once instantiated), after which the results of instantiation are displayed on the command line. Once instantiation is executed, the user can interact with the simulate button, which will again show output on the command line.

### Debug
**![](https://lh4.googleusercontent.com/Plh1yAE1Sx3wgVY3w-fHRgmEh1ZyY2uCe0O2SX1984lYe4kgiUJH-C6_2aLZngWX0-eraXf8m--xC--ouErySfQJFGbAkLe-TuzNa3O5QKRf6F-UAThoJ5oyWi0KRsgLLH6LQVU2)**

By pressing the Debug button on the toolbar, the debug window will appear on the right side of the GUI. There will be two checkboxes: "Log to File" and "Log to Stdout," with the former being set automatically. Logging to file will send debug and error messages to a file which can be renamed at any time in the text below. Logging to stdout will print these messages in the Terminal application running the GUI. Below these options is a table full of debug flags that are native to the gem5 system. These flags can be set and unset, and result in gem5 debug messages related to the flag set. They are also searchable by an accompanying search bar.

### Import and Export UI Objects

**![](https://lh3.googleusercontent.com/siEBwmcd6tGS7QHymwpDtJK7Is0zEQFH30jnCnhSTcjVKfo7rD3oPJskbw_Cty_lu05ifpWIkkwO3wKJqMoDKqtL7XqrEgVqwAXp9X57DmRIjZqkMRpErWt1kLeJYvXZ9Qn2YftK)**

Under file, we have import and export UI object, which allow users to save and load clusters of SymObjects. Exporting saves the configuration as a .obj file, which has a JSON format. Importing places the custom object in the catalog, allowing for the same access methods as regular SimObjects.

### Import SimObject

**![](https://lh6.googleusercontent.com/4ZZ0gDvbqz6o9YIB4-pHTZUBB2Cpxwd7CnC2szd_-dy-fO88NMUCzU1I6YmRTR_oSYgk5wgHl8gce3AAXd3RF7iRJcIZ9mU-qgJyBx13WN-7Ploe03yXLUQHQBnSpfUOaNo-AJoB)**

Users can import custom SimObjects, which are python classes inherited from base SimObjects. These imports appear under the catalog and can be similarly added to the GUI just like regular objects. Imported SimObjects are saved as part of a .ui file, so you do not need to import again when opening a .ui file with imported SimObjects.

## Future

Over the development timeline, we faced roadblocks and came up with new ideas, so we were not able to accomplish everything we initially planned to do. Future development can address these features, as well as others that may be desired:

- Export to multiple file formats (current instantiation generates config.json)
- Visualization of simulation results, comparison to other simulations
- Parameterize objects to allow for running multiple simulations in parallel and comparing results / identifying optimal parameter values

Although we did our best to address bugs we came across during development and user testing, the fact of the matter is a sandbox application with few restrictions on the user such as this GUI will yield numerous bugs through unforeseen usage. If you come across a bug let us know by opening an [issue](https://github.com/afarooqui98/gem5_GUI/issues), submitting a [pull request](https://github.com/afarooqui98/gem5_GUI/pulls), or contacting us or the gem5 team directly. Please try to document the steps resulting in a fault, as well as including a screenshot of the terminal.
