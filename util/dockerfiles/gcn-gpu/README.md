## gcn3-gpu dockerfile
This dockerfile contains all the dependences necessary to run GPU applications in gem5 using the gcn3 APU model

### Building the image
```
docker build -t <image_name> .
```

### Building gem5 using the image
The following command assumes the gem5 directory is a subdirectory of your current directory
```
docker run --rm -v $PWD/gem5:/gem5 -w /gem5 <image_name> scons -sQ -j$(nproc) build/GCN3_X86/gem5.opt
```

### Test gem5 using a prebuilt application
```
wget http://dist.gem5.org/dist/current/test-progs/hip_sample_bins/MatrixTranspose
docker run --rm -v $PWD/MatrixTranspose:/MatrixTranspose -v $PWD/public_gem5:/gem5 -w /gem5 \
        <image_name> build/GCN3_X86/gem5.opt configs/example/apu_se.py -n2 --benchmark-root=/ -cMatrixTranspose
```

### Notes
* When using the `-v` flag, the path to the input file/directory needs to be the absolute path; symlinks don't work
* Currently linking in an AFS volume is not supported, as it uses ACLs instead of owner/group IDs

### ToDo
* Add square to gem5-resources github, add directions for building and running an application
