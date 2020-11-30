This is the gem5 simulator.

The main website can be found at http://www.gem5.org

A good starting point is http://www.gem5.org/about, and for
more information about building the simulator and getting started
please see http://www.gem5.org/documentation and
http://www.gem5.org/documentation/learning_gem5/introduction.

To build gem5, you will need the following software: g++ or clang,
Python (gem5 links in the Python interpreter), SCons, SWIG, zlib, m4,
and lastly protobuf if you want trace capture and playback
support. Please see http://www.gem5.org/documentation/general_docs/building
for more details concerning the minimum versions of the aforementioned tools.

Once you have all dependencies resolved, type 'scons
build/<ARCH>/gem5.opt' where ARCH is one of ARM, NULL, MIPS, POWER, SPARC,
or X86. This will build an optimized version of the gem5 binary (gem5.opt)
for the the specified architecture. See
http://www.gem5.org/documentation/general_docs/building for more details and
options.

The basic source release includes these subdirectories:
   - configs: example simulation configuration scripts
   - ext: less-common external packages needed to build gem5
   - src: source code of the gem5 simulator
   - system: source for some optional system software for simulated systems
   - tests: regression tests
   - util: useful utility programs and files

To run full-system simulations, you will need compiled system firmware
(console and PALcode for Alpha), kernel binaries and one or more disk
images.

If you have questions, please send mail to gem5-users@gem5.org

Enjoy using gem5 and please share your modifications and extensions.

# 关于分支的使用

首先，使用如下命令clone该仓库
```
git clone https://github.com/Raymondwo/gem5-src.git
```

然后，在stable分支的基础上，创建自己的分支。
```
git checkout stable
git checkout -b ray-mybranch
```

修改本地文件，例如readme。
```
git add readme
git commmit -m "modify the readme file"
```

上传本地分支到远程。

```
git push  --set-upstream origin ray-mybranch
```


> 关于分支的说明
> 所有ray创建的分支将使用ray作为开头。首先，大家创建一个自己的master分支，然后在该分支的基础上创建tutorial分支等。

