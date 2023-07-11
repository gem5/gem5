# Installing SST

The links to download SST source code are available here
[http://sst-simulator.org/SSTPages/SSTMainDownloads/].
This guide is using the most recent SST version (11.0.0) as of September 2021.
The following guide assumes `$SST_CORE_HOME` as the location where SST will be
installed.

## SST-Core

### Downloading the SST-Core Source Code

```sh
wget https://github.com/sstsimulator/sst-core/releases/download/v11.1.0_Final/sstcore-11.1.0.tar.gz
tar xf sstcore-11.1.0.tar.gz
```

### Installing SST-Core

```sh
cd sstcore-11.1.0
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
wget https://github.com/sstsimulator/sst-elements/releases/download/v11.1.0_Final/sstelements-11.1.0.tar.gz
tar xf sstelements-11.1.0.tar.gz
```

### Installing SST-Elements

```sh
cd sst-elements-library-11.1.0
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

At the root of gem5 folder,

```sh
scons build/RISCV/libgem5_opt.so -j $(nproc) --without-tcmalloc --duplicate-sources
```

**Note:** `--without-tcmalloc` is required to avoid a conflict with SST's malloc.
`--duplicate-sources` is required as the compilation of SST depends on sources to be present in the "build" directory.

### Compiling the SST integration

At the root of gem5 folder,

```sh
cd ext/sst
make
```

### Running an example simulation

See `README.md`
