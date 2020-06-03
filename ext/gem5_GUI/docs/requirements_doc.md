- [GUI for GEM5](#gui-for-gem5)
      - [Ahmed Farooqui, Ravishdeep Singh, Rohit Dhamankar, Shivam Desai](#ahmed-farooqui--ravishdeep-singh--rohit-dhamankar--shivam-desai)
      - [University of California, Davis](#university-of-california--davis)
  * [Introduction](#introduction)
  * [Glossary of Terms](#glossary-of-terms)
  * [System Architecture](#system-architecture)
  * [Requirements](#requirements)
  * [Technologies Employed](#technologies-employed)
  * [Cost Analysis](#cost-analysis)
  * [Social and Legal Considerations](#social-and-legal-considerations)
- [Technology Survey](#technology-survey)
  * [Virtual Machine](#virtual-machine)
  * [Linux Distribution](#linux-distribution)
  * [GUI](#gui)
  * [Project Management Survey](#project-management-survey)
  

# GUI for GEM5

#### Ahmed Farooqui, Ravishdeep Singh, Rohit Dhamankar, Shivam Desai
#### University of California, Davis

https://github.com/afarooqui98/ECS193a/edit/master/docs/requirements_doc.md

## Introduction

Currently, gem5 is a highly active open source project that simulates different experiments in Computer Architecture. The maintainers of gem5 originally developed this project for computer architecture research in academia. Given the scope of this project, however, this system can be used for computer system design, industry research, and for students in undergraduate architecture classes. The current software allows users to configure architectural models through python scripts. To make gem5 more accessible and allow new users to utilize all of its capabilities, we want to create a user interface that will allow such ease of use and functionality. Our goal is to develop this front-end GUI integrated with gem5 in order to make configuring systems much more efficient.

## Glossary of Terms
* gem5 - the current open source project used by research to configure computer architecture systems.
* SimObject - "Simulation Object", This relates to a series of classes that can be instantiated in order to represent an architectural component. Each SimObject has its own paramters which can be individually configured at instantiation and connected to other SimObjects to form a system. Examples include CPU, Cache, etc.

## System Architecture
![Sample Mockup for GUI](https://lh3.googleusercontent.com/Mt63W_M_vAmhWoj0TZRSEkX4HImpBlKc7CEHHW4LY3DWzmJIfQ8jLgpQIFRFAdGKx3nag5z9u1npWYqJejgRaT-A8lkSlHBF_XYKfpKS-qpZQwPz0ZaTyY9npvaBcDvN6mKt_2k)

![GUI Architecture](https://lh3.googleusercontent.com/kXC5Z-s27iuzWKmvDEteqzpl5c_CSPyXb64dq5v3ByAwsI4BJQYWbpUQDgBn-tpGGpTHFu6-sGjTOC561eeE-H60Ad1Z6dYwmvcti_9FozA8hcufProGifHlnXt-axfMeDlKz8aPqNUVIz5m_mYsUrSIR1Nkjxph_kw_kiOU3fpIXMLOG_AJcOXehANCWM1h0_fGSSmhjwvuDrbd0ZdCbKi83C9QfPeOwO0k6SjzylrcMQgC-AfaEEk7inIj0UlbFUDHW9CRz_mQ6E5EuGuep5ib6iPHvCFxrOjWSGEEMlrNFjpE0KhBzuT-vRrBS9dW7wQlNLNdUrxZB7ajLZiYVaEv0TDp__0SqlSyn0k_66sJgqvoiu_pubRi8nuYVztEm_xeC1HWdnf1-y8l4F5b17PgsD7S8co71BXyNtV05YdRDRKHoKsxaEk1HpnVj9-35w_YrrUiy0zDwKPVJTmxHKAHxZ4U1lnlBT5OLbQgYqNd0-xmnWFY8kLRVemLqub-nDKeNzXiIBK73mnqpH4rYW9-IqJ1esGcD7REYCc0tVsGFG2Yq7WIHYdudNerD8xMprCQKvAHvghJeTtK5fbE-JTf5j23HRIcShKT1Xel0aIazgk7FEsOH6J5_2q8MAZHfazUh81Z9EdZ1HjzZGZ-GccupJRjapWT-SJYVSzdMLxct9WZsZ_oGaVrkKTk=w1754-h1241-no)

Our primary architecture will be based on this general design mockup. There is a logical front and back end to this system, but physically they may exist in the same space. The categories are as follows:

Front-end: The primary UI interaction and design comprise the front end. Dragging and dropping objects, selecting them, and highlighting multiple objects are all part of the front-end.

Back-end: The front and back-end are quite tightly coupled, as all the objects that are draggable and selectable are mapped to what are called "SimObjects" which define the schema and function of a particular computer architectural entity, like a CPU, for example. This list of objects will be colloquially known as the "catalog" which dynamically loads all of the SimObjects at program initialization and allows users to select the component they want from a categorized and searchable menu. Each selected object will be its own instance of a specific SimObject, with varying parameters, some of which have default values and some that need to be filled in. The description of the objects and their parameters are part of this category. Furthermore, users will commonly use this tool to save their work and subsequently export to a file format suited for running in the gem5 simulation environment. Finally, users will be able to save and load collections of objects within the GUI and then pull them from the aforementioned catalog, in a separate section dedicated to user collections.

## Requirements

Must Haves:

* Users can drag and drop new SimObjects into a model and connect them with wires.

    * Each SimObject has a GUI representation which is then stored in what we’ll refer to as a "catalog"
	 		
		* These objects will be generated dynamically from the current SimObjects available in gem5
		
		* Users can "import" more objects by pointing to a python file with SimObject classes defined
		
		* SimObjects will be initialized in the GUI, which will allow users to modify the parameters for a specific instance of that SimObject
			
    * An object can be selected, which will bring up its modifiable parameters which will allow the user the change them as necessary

    * Connecting two objects is a matter of dragging a wire from one **port** to another

* As a user, I can easily modify simulation object parameters to easily configure and test different systems

    * Some parameters will have default settings based on the type of object selected

    * In the attribute selection view hovering over each parameter should display a description of that parameter

* Users can save model files and access them later to make cross system development easier.

* As a researcher, I can easily design and test new architectures to shorten the development time.

    * The software should be able to manage multiple designs and load multiple files without any significant performance hit. The most expensive operation should happen at file export, where (ideally) code is generated for a user to run with the gem5 compiler.

Should Haves:

* Users can export the model into a JSON file format

    * The current plan is to allow the user to export to different files formats, like a python configuration script, for the obvious purpose of allowing users to continue development in Python if necessary. The actual simulator will be a tertiary feature that we’ll account later down the line

* Users can create new simulation objects derived from existing objects to develop new architectures.

* As a user I will be able to select multiple objects and save them to the aforementioned catalog.

* Users can use shortcut keys (hot keys) to perform common tasks such as selecting a wire or run simulation.

    * Copy, Paste, Cut, multiple selection (will not bring up an attribute screen but will allow multiple objects to be moved around)

Could Haves:

* Besides a JSON file, users can export a design to multiple file formats, primarily to a python configuration script that can be essentially immediately run .

* As a user, I can visualize simulation results and compare results between different simulations and architectures to easily compare and contrast different systems.

* Users can run simulations from the GUI directly via a one click simulation button to shorten development time.

* Parameterize objects and run multiple simulations in parallel over these object parameters in order to determine the optimal configurations.

## Technologies Employed
Our GUI will be implemented in Python, using the GUI development framework PyQt5. For the development process, we will use a linux virtual machine running Ubuntu 18.04. We will study the existing gem5 code base to maintain consistency between the GUI and the backend software. We will also be using JIRA as our product management tool to develop tasks as well as communicate with our client and other gem5 contributors.

## Cost Analysis
Since there is no hardware costs and all the technologies are free to use, we have no monetary costs to consider.

## Social and Legal Considerations
Our GUI for gem5 will be an open source project. This means that a multitude of users can use, modify, and share the work. Currently, all the files in the gem5 distribution have licenses based on BSD or MIT license. For our GUI we will be employing a modified BSD license.

# Technology Survey

## Virtual Machine
   Parallels
   
   VirtualBox
   
   VMWare Workstation

## Linux Distribution
Ubuntu

Fedora

Red Hat

Debian

## GUI 

* Qt

	* Pros:
		* Cross-platform

    	* Suggested by client
        
    	* Extensively used by many organizations
    
    	* Lots of documentation and tutorials

	* Cons:
    	
        * No experience using it

* Kivy

    * Pros:

        * Cross-platform

        * Written in C and graphics engine is built in OpenGL Es2
    
        * Well integrated with Pycharm

        * Free to us under MIT license

	* Cons:
		* No experience using it

        * More suitable for developing games not really used for desktop apps

* GTK:

    * pros:

    * cross-platform
    
    * written in c, but has support for other languages
    
    * no licensing restrictions
    
    * cons:

        * no experience using it

* TKinter: (python)

    * pros:

        * simple, open source

        * bundled with python

        * lots of resources / tutorials

    * cons:

        * not as aesthetic as other technologies



## Project Management Survey

Conclusion: We will be using JIRA since the existing gem5 team uses JIRA.
