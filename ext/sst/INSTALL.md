# Installing SST

The links to download SST source code are available at
<http://sst-simulator.org/SSTPages/SSTMainDownloads/>.
This guide is using the most recent SST version (13.0.0) as of September 2023.
The following guide assumes `$SST_CORE_HOME` as the location where SST will be
installed.

## SST-Core

### Downloading the SST-Core Source Code

```sh
wget https://github.com/sstsimulator/sst-core/releases/download/v13.0.0_Final/sstcore-13.0.0.tar.gz
tar xvf sstcore-13.0.0.tar.gz
```

### Installing SST-Core

```sh
cd sstcore-13.0.0
./configure --prefix=$SST_CORE_HOME --with-python=/usr/bin/python3-config \
            --disable-mpi # optional, used when MPI is not available.
make all -j$(nproc)
make install
```

Update `PATH`,

```sh
export PATH=$SST_CORE_HOME/bin:$PATH
```

## SST-Elements

### Downloading the SST-Elements Source Code

```sh
wget https://github.com/sstsimulator/sst-elements/releases/download/v13.0.0_Final/sstelements-13.0.0.tar.gz
tar xvf sstelements-13.0.0.tar.gz
```

### Installing SST-Elements

```sh
cd sst-elements-library-13.0.0
./configure --prefix=$SST_CORE_HOME --with-python=/usr/bin/python3-config \
            --with-sst-core=$SST_CORE_HOME
make all -j$(nproc)
make install
```

Adding `PKG_CONFIG_PATH` to `.bashrc` (so pkg-config can find SST .pc file),

```sh
echo "export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$SST_CORE_HOME/lib/pkgconfig/" >> ~/.bashrc
```

### Building gem5 library

At the root of the gem5 folder, you need to compile gem5 as a library. This
varies which OS you use. If you're using Linux, then type the following:
```sh
scons build/RISCV/libgem5_opt.so -j $(nproc) --without-tcmalloc --duplicate-sources
```
In case you're using Mac, then type the following:
```sh
scons build/RISCV/libgem5_opt.dylib -j $(nproc) --without-tcmalloc --duplicate-sources
```

**Note:**
* `--without-tcmalloc` is required to avoid a conflict with SST's malloc.
* `--duplicate-sources` is required as the compilation of SST depends on sources to be present in the "build" directory.
* The Mac version was tested on a Macbook Air with M2 processor.

### Compiling the SST integration

Go to the SST directory in the gem5 repo.
```sh
cd ext/sst
```
According to the OS that you're using, you need to rename the `Makefile.xxx` to `Makefile`.
```sh
cp Makefile.xxx Makefile    # linux or mac
make -j4
```
Change `ARCH=RISCV` to `ARCH=ARM` in the `Makefile` in case you're compiling
for ARM.
### Running an example simulation

See `README.md`
