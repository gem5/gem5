## pyfdt : Python Flattened Device Tree Manipulation ##
----------
The pyfdt library is aimed to facilitate manipulation of the flattened device tree in order to parse it and generate output in various formats.

It is highly based on fdtdump for the dtc compiler package.

Support Inputs :
 - Device Tree Blob (.dtb)
 - Filesystem
 - JSON (See JSONDeviceTree.md)

Supported Outputs :
 - Device Tree Blob (DTB)
 - Device Tree Structure (text DTS)
 - JSON (See JSONDeviceTree.md)

Device Tree filesystem 'output' is available via the fusemount.py FUSE sample using [fusepy](https://github.com/terencehonles/fusepy) library.

The object data permits :
 - add/delete/pop nodes and attributes
 - create attributes dynamically with native python types
 - walk throught the tree
 - resolve and generate "paths"
 - parse from DTB or filesystem
 - output DTB or DTS
 - output JSON
 - compare two tree
 - merge two trees

Any API, code, syntax, tests or whatever enhancement is welcomed, but consider this an alpha version project not yet used in production.

No DTS parser/compiler is event considered since "dtc" is the official compiler, but i'm open to any compiler implementation over pyfdt...

Typical usage is :
```
from pyfdt import FdtBlobParse
with open("myfdt.dtb") as infile:
    dtb = FdtBlobParse(infile)
    print dtb.to_fdt().to_dts()
```

Will open a binary DTB and output an human readable DTS structure.

The samples directory shows how to :
 - checkpath.py : resolve a FDT path to get a node object
 - dtbtodts.py : how to convert from DTB to DTS
 - fusemount.py : how to mount the DTB into a Device Tree filesystem you can recompile using dtc
 - python-generate.py : generate a FDT in 100% python and generate a DTS from it
 - walktree.py : List all paths of the device tree
 - fstodtb.py : Device Tree blob creation from Filesystem device tree like DTC

[Device Tree Wiki](http://www.devicetree.org)
[Device Tree Compiler](http://www.devicetree.org/Device_Tree_Compiler)

[![Build Status](https://travis-ci.org/superna9999/pyfdt.svg?branch=master)](https://travis-ci.org/superna9999/pyfdt)
