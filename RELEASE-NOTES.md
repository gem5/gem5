# Version 20.0.0.0

* Compiling and running gem5 with Python 3 is now fully supported.
* Compiling and running gem5 with GCC 8 and 9 is now supported.
* Scons-based tests have been migrated to the testlib framework. Please consult TESTING.md for more information on how these may be run.
* Support for the ALPHA ISA has been dropped.
* Memory SimObjects can now be initialized using an image file using the image_file parameter.
* The m5 utility has been revamped with a new build system based on scons, tests, and updated and more consistent feature support.
* Robust support for marshalling data from a function call inside the simulation to a function within gem5 using a predefined set of rules.
* Workload configuration pulled out into its own object, simplifying the System object and making workload configuration more modular and flexible.
