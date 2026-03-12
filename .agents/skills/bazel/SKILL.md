---
name: gem5-bazel-build
description: Guides the creation and management of Bazel BUILD files for gem5 components, including C++, Python, SimObjects, and Enums. Use when working with gem5 BUILD files or translating from SCons.
---

# How-To: Create gem5 Build Targets with Bazel

This manual provides a step-by-step guide to creating build targets for various gem5 components using the Bazel build system.

## When to use this skill

-   When you need to create or modify `BUILD` files within the gem5 project.
-   When translating gem5 build configurations from SCons to Bazel.
-   When debugging build issues related to gem5's custom Bazel rules.
-   When adding new C++ or Python code, SimObjects, or Enums to the gem5 build system.


This manual provides a step-by-step guide to creating build targets for various gem5 components using the Bazel build system.

## 1. Core Concepts

The gem5 Bazel build system uses a set of custom rules defined in `bazel/rules.bzl`. The primary rules are:

-   `gem5_cc_library`: For compiling C++ source code. It automatically adds the necessary include paths for gem5.
-   `py_library`: The standard Bazel rule for Python libraries.
-   `gem5_sim_object`: The core rule for creating SimObjects. It generates the C++ parameter-handling code that bridges the Python configuration and the C++ implementation.
-   `gem5_enum_generator`: A specialized rule for handling Python `Enum` classes that need to be exposed to C++.
-   `gem5_cc_test`: A wrapper for the standard `cc_test` rule for running C++ unit tests.
-   `gem5_isa_cc_library`: An advanced rule for parsing `.isa` files and generating the C++ implementation for an Instruction Set Architecture.
-   `gem5_debug_flag`: For defining debug flags used with `DPRINTF`.

## 2. Overall Methodology & Strategy

*   **Dependency First:** Always start by identifying and creating BUILD files for the lowest-level dependencies. Work your way up the dependency chain. Use `bazel query` or analyze import statements (`#include` in C++, `import` in Python) to understand dependencies.
*   **Iterative Approach:** Don't try to create the entire perfect BUILD file at once. Add targets incrementally and test frequently using `bazel build`.
*   **One Target at a Time:** When encountering build errors, especially in a new BUILD file, build targets individually to isolate the problematic one.
*   **Strict Adherence to This Guide:** Follow the prescribed patterns for each file type.

**Order of Operations:**

1.  **Create Package:** Ensure the directory has a `BUILD` file. Start with a basic package definition.
2.  **Debug Flags:** Add all `gem5_debug_flag` targets. These are usually self-contained within the package.
3.  **Python Libraries (`py_library`):** Create targets for all `.py` files, adding dependencies as needed.
4.  **Shim Registry (`sim_objects.bzl`):** If a Python file defines a SimObject that should be importable via `from m5.objects import ...`, ensure it's registered in `bazel/sim_objects.bzl`.
5.  **Enum Generators (`gem5_enum_generator`):**
    *   These depend on the `py_library` containing the enum definition.
    *   Crucially, they also need any dependencies required to *parse* that Python file in the `extra_tool_deps`. This includes any modules imported via the `m5.objects` shim.
6.  **C++ Header Libraries (`gem5_cc_library` name = "..._hh"):** Create these first, as implementation targets depend on them.
7.  **C++ Implementation Libraries (`gem5_cc_library`):** Create targets with `srcs` and `deps` on the `_hh` targets and other dependencies.
8.  **SimObject Generators (`gem5_sim_object`):**
    *   These generate parameter header targets (`:param_<SimObject>_hh`) and an alias target (`:<sim_object_name>_sim_object`).
    *   The `name` should follow the convention `:<sim_object_name>_sim_object`.
    *   `sim_object_py_lib` points to the `py_library`.
    *   `sim_object_header` points to the C++ header-only target.
    *   `extra_deps` includes C++ implementation dependencies.
    *   `extra_param_deps` includes dependencies needed by the generated parameter header. These should use the `_sim_object` suffix when referring to other SimObjects.
    *   `extra_tool_deps` includes Python dependencies needed to parse the SimObject Python file, similar to enum generators.
9.  **C++ Tests (`gem5_cc_test`):** Add tests for your C++ libraries.

## 3. Tool Usage

*   **`view_file <path>`**: ALWAYS read a file before editing it to understand its current state and to provide correct `TargetContent` values for the `replace_file_content` tool.
*   **`write_to_file <path> <content>`**: Use for creating new BUILD files or completely overwriting existing ones when the changes are substantial.
*   **`replace_file_content <path> <instruction> <target_content> <replacement_content>`**: Use for targeted modifications. Be VERY precise with `target_content`, including line breaks and indentation.
*   **`run_command "bazel build ..."`**: Build targets frequently to catch errors early.
*   **`run_command "bazel query <query>"`**: Extremely useful for debugging:
    *   `bazel query //path/to/package:all`: List all targets in a package.
    *   `bazel query "deps(//path/to:target)"`: Show dependencies of a target.
    *   `bazel query "somepath(//path/to:target1, //path/to:target2)"`: Find a dependency path.
*   **`read_terminal <command_id>`**: Check the output of background commands.
*   **`run_command "bazel clean --expunge"`**: Use sparingly if you suspect Bazel cache issues are causing unexpected behavior.

## 4. How-To Guides & Target Creation

### 4.1. Creating a Basic C++ Library

This is the most common task. It involves creating `gem5_cc_library` targets. The best practice for **new targets** is to split them into a header-only version and a full implementation to improve modularity and reduce recompilation times.

**Scenario:** You have created `my_util.hh` and `my_util.cc` and need to create new build targets for them.

**Steps:**

1.  **Edit the `BUILD` file** in the same directory.
2.  **Create the header-only `gem5_cc_library` target:** This target exposes only the declarations from `my_util.hh`.

    ```python
    gem5_cc_library(
        name = "my_util_hh", # Suffix with _hh for header-only
        hdrs = ["my_util.hh"],
        deps = [
            # Add dependencies based on #includes in my_util.hh
            # e.g., "//src/base:types",
        ],
    )
    ```

3.  **Create the full implementation `gem5_cc_library` target:** This target includes the source file and depends on its header-only counterpart.

    ```python
    gem5_cc_library(
        name = "my_util", # No _hh suffix for the full implementation
        srcs = ["my_util.cc"],
        # Do NOT include "my_util.hh" in hdrs if it is already in :my_util_hh
        # This avoids layering check violations.
        deps = [
            ":my_util_hh", # Depend on the header-only version
            # Add any *additional* dependencies specific to my_util.cc
            # e.g., "//src/base:random",
        ],
    )
    ```

**Note on Existing Targets:** If you are adding a dependency on a library that already has a `BUILD` target, use the existing target name (e.g., `//src/base:logging`) even if it is not split into an `_hh` version. **Do not refactor existing targets.**

### 4.2. Adding a C++ Unit Test

**Scenario:** You have created `my_util.test.cc` to test your `my_util` library.

**Steps:**

1.  **Edit the `BUILD` file.**
2.  **Add a `gem5_cc_test` target:**

    ```python
    gem5_cc_test(
        name = "my_util_test",
        srcs = ["my_util.test.cc"],
        deps = [
            ":my_util",  # The library to be tested
            "//testing/base/public:gunit_main",
        ],
    )
    ```

### 4.3. Creating a New SimObject

This is a multi-step process involving a Python library, C++ libraries, and the generator macro.

**Scenario:** You are creating a new SimObject named `MyObject`, defined in `MyObject.py` and implemented in `my_object.hh` and `my_object.cc`. `MyObject` inherits from `ClockedObject`.

**Step 1: Create the `py_library` for the Definition**

```python
# In your BUILD file
py_library(
    name = "MyObject_py",
    srcs = ["MyObject.py"],
    deps = [
        # Map your Python imports to deps.
        # e.g., for "from m5.objects import ClockedObject"
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
        "//src/python/m5:proxy",
        "//src/python/m5:SimObject",
    ],
)
```

**Step 2: Create the `gem5_cc_library` Targets for the Implementation**

Following the best practice for new C++ libraries, create two `gem5_cc_library` targets: one for the header and one for the source.

1.  **Header-Only C++ Library (`my_object_hh`):**

    ```python
    # In your BUILD file
    gem5_cc_library(
        name = "my_object_hh",
        hdrs = ["my_object.hh"],
        deps = [
            # This target is created by the gem5_sim_object macro
            ":MyObject_sim_object",
            # Depend on the header-only version of the parent class's C++ impl
            "//src/sim:clocked_object_hh",
        ],
    )
    ```

2.  **Full C++ Implementation Library (`my_object`):**

    ```python
    # In your BUILD file
    gem5_cc_library(
        name = "my_object",
        srcs = ["my_object.cc"],
        hdrs = ["my_object.hh"],
        deps = [
            ":my_object_hh",
            # Add any other dependencies specific to the .cc file
        ],
    )
    ```

**Step 3: Create the `gem5_sim_object` Bridge**

This rule ties everything together.

```python
# In your BUILD file
gem5_sim_object(
    name = "MyObject_sim_object",
    # ---
    # Core Parameters ---
    # ---
    sim_object_py_lib = ":MyObject_py",
    sim_object_name = "MyObject",
    sim_object_module = "src.path.to.your.module.MyObject", # Python import path
    sim_object_header = ":my_object_hh", # Link to the C++ header-only lib

    # ---
    # Dependency Parameters ---
    # ---
    # For Python dependencies of the generator tool
    extra_tool_deps = [
        # Add the py_library for the parent class
        "//src/python/m5/objects:ClockedObject",
        # Add any other SimObjects used as Param types
    ],
    # For C++ header dependencies of the generated code
    extra_param_deps = [
        # Add the generated target for the parent class
        "//src/sim:ClockedObject_sim_object",
        # Add any other *_sim_object for SimObjects used as Param types
    ],
    # For C++ implementation dependencies of the generated code
    extra_deps = [
        ":my_object",
    ],
)
```

**Step 4: Update `sim_objects.bzl`**

Finally, register your new SimObject by adding an entry to the `SIM_OBJECTS` list in `bazel/sim_objects.bzl`.

**Distinction between `simobjects` and `other_simobjects`:**

*   **`simobjects`**: Use this list for SimObject classes that have a corresponding C++ implementation (i.e., they have a `gem5_sim_object` macro). These entries are used to generate C++ binding dependencies.
*   **`other_simobjects`**: Use this list for SimObject classes defined in the Python file that **do not** have a direct C++ equivalent. These are typically Python subclasses of another SimObject that reuse the parent's C++ implementation. They do not require their own `gem5_sim_object` macro.

**Workflow for Categorization:**

To determine which list a class belongs to:

1.  **Inspect the Python file (`MyObject.py`):** Identify all classes that inherit from `SimObject` (or its subclasses) and all classes that inherit from `Enum`.
2.  **Inspect the C++ files/BUILD targets:** Check if there is a corresponding C++ implementation (usually a `.cc` file and a `gem5_sim_object` macro) for each SimObject class found.
3.  **Categorize:**
    *   If a SimObject class has a C++ implementation, add it to `simobjects`.
    *   If a SimObject class is Python-only (reuses a parent's C++), add it to `other_simobjects`.
    *   Add all Enums to `enums`.

```python
    {
        "mod_name": "MyObject",
        "module_path": "src.path.to.your.module.MyObject",
        "dep": "//src/path/to/your/module:MyObject_py",
        "simobjects": ["MyObject"],
        "enums": [], # Add any enums defined in MyObject.py
        "other_simobjects": [],
    },
```

**Note on Subclasses:** If you define a class in the Python file that extends another SimObject (but isn't the primary SimObject of the module), you must list it in the `other_simobjects` field. This ensures the build system registers it correctly.

### 4.4. Adding a New Enum

**Scenario:** You have a Python file `MyEnums.py` that contains `enum class MyEnum(Enum): ...`.

**Step 1: Create a `py_library`**

```python
py_library(
    name = "MyEnums_py",
    srcs = ["MyEnums.py"],
    deps = [
        "//src/python/m5/params",
    ],
)
```

**Step 2: Create the `gem5_enum_generator`**

```python
gem5_enum_generator(
    name = "enum_MyEnum",
    enum_name = "MyEnum",
    enum_module = "src.path.to.your.module.MyEnums",
    enum_py_lib = ":MyEnums_py",
    extra_tool_deps = [
        "//src/python/m5/params",
    ],
)
```
This will generate `:enum_MyEnum_hh` and `:enum_MyEnum_py` targets that you can depend on from C++ and Python respectively.

## 3. Dependency Mapping Guide

### 3.1. Python `import` to `deps`

| Python `import` statement | Bazel `deps` Target |
| --------------------------- | --------------------- |
| `from m5.objects import <SimObject>` | `//src/python/m5/objects:<SimObject>` |
| `from m5.params import *` | `//src/python/m5/params` |
| `from m5.proxy import *` | `//src/python/m5:proxy` |

### 3.2. C++ `#include` to `deps`

The `gem5_cc_library` rule automatically adds `src/` to the include search path. Therefore, you can map `#include` directives directly to Bazel targets.

| C++ `#include` statement | Bazel `deps` Target |
| ------------------------ | ------------------- |
| `#include "base/types.hh"` | `//src/base:types"` |
| `#include "sim/clocked_object.hh"` | `//src/sim:clocked_object_hh"` |
| `#include "mem/cache/cache.hh"` | `//src/mem/cache:cache"` |
| `#include "path/to/local_header.hh"` | `//path/to:local_header` or `:local_header`|
| `#include "params/MyObject.hh"` | `:MyObject_sim_object` (This is a generated target) |

---

## 4. Mapping SCons to Bazel

This section provides a detailed guide on how to translate gem5's legacy SCons build definitions (`SConscript` files) into the modern Bazel (`BUILD`) format.

### 4.1. General Methodology

The core difference is that SCons uses an *imperative* and *discovery-based* approach (you declare a source, and SCons figures out dependencies implicitly), while Bazel uses a *declarative* and *explicit* approach (you must declare all targets and their dependencies).

To translate a directory's build instructions, you must consult three sources of information:

1.  **The `SConscript` file:** This is your "table of contents." It tells you *what* needs to be built (SimObjects, C++ sources, debug flags, etc.). It is the starting point for identifying the components that require a Bazel target.
2.  **The Source Code (`.py`, `.cc`, `.hh`):** This is where you find the *explicit dependencies* that are hidden in SCons. You must read the `import` statements in Python files and the `#include` statements in C++ files to determine the `deps` for your Bazel targets.
3.  **The `sim_objects.bzl` file:** This is the central registry for all SimObjects and their associated Enums, located at `bazel/sim_objects.bzl`. **Any SimObject or Enum that is part of the build must have an entry here.** This is a critical step that SCons does not have.

### 4.2. SCons to Bazel Rule Mapping

Here is a direct mapping of the most common SCons functions to their Bazel equivalents.

---

#### **`SimObject(...)`**

This is the most complex mapping, as a single SCons line translates into multiple, interconnected Bazel targets.

**SCons (`SConscript`):**
```python
SimObject('MyObject.py', sim_objects=['MyObject'], enums=['MyEnum'])
```

**Bazel (`BUILD` file):**

This one SCons line requires **four** Bazel targets:

1.  A **`py_library`** for the Python definition:
    ```python
    py_library(
        name = "MyObject_py",
        srcs = ["MyObject.py"],
        deps = [
            # ... Dependencies derived from `import` statements in MyObject.py
            # e.g., "//src/python/m5/objects:ParentSimObject"
        ],
    )
    ```

2.  A **`gem5_enum_generator`** for each enum listed (if any):
    ```python
    gem5_enum_generator(
        name = "enum_MyEnum",
        enum_name = "MyEnum",
        enum_module = "src.[...].MyObject", # Full Python path to module
        enum_py_lib = ":MyObject_py",
    )
    ```

3.  A **`gem5_sim_object`** for each SimObject class listed:
    ```python
    gem5_sim_object(
        name = "MyObject_sim_object",
        sim_object_py_lib = ":MyObject_py",
        sim_object_name = "MyObject",
        sim_object_module = "src.[...].MyObject",
        sim_object_header = ":my_object", # Points to the C++ impl library
        extra_tool_deps = [ # Python deps of the generator (from imports)
            "//src/python/m5/objects:ParentSimObject",
        ],
        extra_param_deps = [ # C++ deps for the generated header (parent classes, enums)
            "//path/to/parent:ParentSimObject_sim_object",
            ":enum_MyEnum_hh",
        ],
    )
    ```

4.  **Registration in `sim_objects.bzl`:** This step has no SCons equivalent and is mandatory.
    ```python
    # In bazel/sim_objects.bzl
    {
        "mod_name": "MyObject",
        "module_path": "src.[...].MyObject",
        "dep": "//path/to:MyObject_py",
        # Classes with a C++ implementation (have a gem5_sim_object macro)
        "simobjects": ["MyObject"],
        # Classes defined in MyObject.py but without their own C++ implementation
        # (e.g., Python-only subclasses reusing the parent's C++)
        "other_simobjects": [],
        "enums": ["MyEnum"],
    },
    ```
    ```

---

#### **`Source(...)`**

This maps to one or two `gem5_cc_library` targets.

**SCons (`SConscript`):**
```python
Source('my_util.cc')
```

**Bazel (`BUILD` file):**

*   **For new files (Best Practice):** Create a split `_hh` and implementation target.
    ```python
    # Header-only target
    gem5_cc_library(
        name = "my_util_hh",
        hdrs = ["my_util.hh"],
        deps = [ # ... deps from includes in the .hh file ],
    )

    # Full implementation target
    gem5_cc_library(
        name = "my_util",
        srcs = ["my_util.cc"],
        hdrs = ["my_util.hh"],
        deps = [
            ":my_util_hh",
            # ... additional deps from the .cc file
        ],
    )
    ```
*   **For existing files:** Match the existing target structure. If a single `gem5_cc_library` exists for both `.hh` and `.cc`, do not change it. Use that single target as a dependency.

---

#### **`DebugFlag(...)` and `CompoundFlag(...)`**

This is a direct, one-to-one mapping.

**SCons (`SConscript`):**
```python
DebugFlag('MyDebugFlag', 'Description')
CompoundFlag('MyCompound', ['MyDebugFlag', 'OtherFlag'])
```

**Bazel (`BUILD` file):**
```python
gem5_debug_flag(
    name = "debug_MyDebugFlag",
    flag = "MyDebugFlag",
    desc = "Description", # Optional
    fmt = False, # Set to True if this flag modifies trace format (e.g. Exec flags)
)

# Compound debug flag
gem5_debug_flag(
    name = "debug_MyCompound",
    flag = "MyCompound",
    components = [
        ":debug_MyDebugFlag",
        "//path/to/other:debug_OtherFlag",
    ],
)
```
**About the `fmt` flag:**
*   **Standard Flags (`fmt = False`):** Most debug flags (e.g., `Fetch`, `Decode`, `Cache`) simply enable or disable a specific set of `DPRINTF` statements. They act as on/off switches for code sections.
*   **Format Flags (`fmt = True`):** These are special flags used primarily by the **Instruction Tracer** (e.g., the `Exec` compound flag). Instead of turning output on/off entirely, they **toggle specific pieces of information** within a trace line (e.g., `ExecAsid`, `ExecEffAddr`). Use `fmt = True` only if you are adding a new option to control the *verbosity or content* of the instruction execution trace.

Add the `debug_*` target to the `deps` of any `gem5_cc_library` using the flag in `DPRINTF`.

**Placement Rule:** Unlike SCons, which often groups debug flags in parent directories (e.g., `src/cpu/SConscript` defining flags for subcomponents), Bazel encourages defining debug flags **in the directory where they are most heavily used**.
*   **Example:** The `Checker` flag is used primarily in `src/cpu/checker`. It should be defined in `src/cpu/checker/BUILD`, not `src/cpu/BUILD`.
*   **Benefit:** This keeps dependencies granular and allows the component to be built and tested in isolation without pulling in unnecessary parent dependencies.

### 4.6. ISA Description Files (`.isa`)

Use `gem5_isa_cc_library` to process `.isa` files.

```python
gem5_isa_cc_library(
    name = "generated_isa",
    arch = "arm", # e.g., 'arm'
    main_isa_file = "isa/main.isa",
    isa_srcs = glob(["isa/**"]),
    constrs_splits = 3,
    exec_splits = 6,
    extra_deps = [ # ... All C++ deps needed by the generated ISA code ],
    extra_hdrs = [ # ... All header deps needed by the generated decoder ],
)
```

---

## 5. Handling Circular Dependencies

Bazel's static dependency analysis will fail if a circular dependency is found (e.g., Target A depends on Target B, and Target B depends on Target A). This can occur if a parent class's Python module imports a child class's module for a shared component like an Enum, or more commonly, due to C++ header includes.

**Python Cycles:**

To resolve this, you must refactor the code by moving the shared component.

**Methodology:**

1.  **Identify the Cycle:** The Bazel error message will clearly show the dependency loop.
2.  **Identify the Shared Component:** Examine the `import` statements in the files involved in the cycle. Find the specific class or Enum that is causing the reverse dependency. For example, `QoSMemCtrl.py` might import from `MemCtrl.py` just to get the `MemSched` Enum.
3.  **Move the Component "Up" the Chain:** Move the shared component from the "child" file into the "parent" file. In the `MemCtrl` -> `QoSMemCtrl` example, `MemCtrl` is the child (it inherits from `QoSMemCtrl`). The `MemSched` Enum should be moved from `MemCtrl.py` into `QoSMemCtrl.py`.
4.  **Update `BUILD` Files:**
    *   Move the `gem5_enum_generator` (or other relevant target) for the shared component from the child's `BUILD` file to the parent's `BUILD` file.
    *   Update the `deps` of any files that were using the old import path.
5.  **Update `sim_objects.bzl`:**
    *   Move the Enum's name from the `enums` list of the child object to the `enums` list of the parent object.

This refactoring breaks the cycle by ensuring dependencies only flow in one direction.

**C++ Header Cycles:**

Another key technique to break cycles, especially between C++ libraries, is to split `gem5_cc_library` targets into header-only (`_hh`) and implementation targets. The `_hh` target contains only the header file and its minimal dependencies (like forward declarations), while the implementation target contains the `.cc` file and depends on the `_hh` target. This allows targets to depend on the header declaration without pulling in the full implementation, breaking potential cycles. You may need to refactor the `.hh` file to remove `#include` statements that cause cycles, potentially moving method implementations to the `.cc` file.

---

## 6. Dependency Lookup Tables

The following tables provide a mapping from common C++ `#include` paths, Python `m5.objects` imports, and `DPRINTF` flags to their corresponding Bazel build targets. **This is not an exhaustive list**, but it covers many of the core components found in the directories analyzed so far.

**Instructions for Use:** When creating `deps` for a new or existing target, use these tables as a reference. Find the `#include` path or `import` statement from your source code in the table to find the correct Bazel target to add to your `deps` list.

### 6.1. C++ Header to Bazel Target Mapping (`gem5_cc_library`)

| `#include` Path | Bazel Target |
| --- | --- |
| **`arch/arm`** | |
| `arch/arm/isa.hh` | `//src/arch/arm:isa` or `:isa_hh` |
| `arch/arm/mmu.hh` | `//src/arch/arm:mmu` or `:mmu_hh` |
| `arch/arm/tlb.hh` | `//src/arch/arm:tlb` or `:tlb_hh` |
| `arch/arm/interrupts.hh` | `//src/arch/arm:interrupts` or `:interrupts_hh` |
| **`base`** | |
| `base/types.hh` | `//src/base:types` |
| `base/logging.hh` | `//src/base:logging` |
| `base/compiler.hh` | `//src/base:compiler` |
| `base/stats.hh` | `//src/base:statistics` or `:statistics_hh` |
| `base/cprintf.hh` | `//src/base:cprintf` |
| `base/random.hh` | `//src/base:random` |
| `base/addr_range.hh` | `//src/base:addr_range` |
| `base/trace.hh` | `//src/base:trace` |
| `base/bitfield.hh` | `//src/base:bitfield` |
| **`cpu`** | |
| `cpu/base.hh` | `//src/cpu:base` or `:base_hh` |
| `cpu/static_inst.hh` | `//src/cpu:static_inst` |
| `cpu/thread_context.hh` | `//src/cpu:thread_context` or `:thread_context_hh` |
| **`mem`** | |
| `mem/abstract_mem.hh` | `//src/mem:abstract_mem` or `:abstract_mem_hh` |
| `mem/mem_interface.hh` | `//src/mem:mem_interface` or `:mem_interface_hh` |
| `mem/mem_ctrl.hh` | `//src/mem:mem_ctrl` or `:mem_ctrl_hh`|
| `mem/packet.hh` | `//src/mem:packet` or `:packet_hh` |
| `mem/port.hh` | `//src/mem:port` or `:port_hh` |
| `mem/request.hh` | `//src/mem:request` |
| **`mem/cache`** | |
| `mem/cache/base.hh` | `//src/mem/cache:base` or `:base_hh` |
| `mem/cache/cache.hh` | `//src/mem/cache:cache` |
| **`mem/protocol`** | |
| `mem/protocol/atomic.hh` | `//src/mem/protocol:atomic` |
| `mem/protocol/functional.hh` | `//src/mem/protocol:functional` |
| `mem/protocol/timing.hh` | `//src/mem/protocol:timing` |
| **`mem/qos`** | |
| `mem/qos/policy.hh` | `//src/mem/qos:policy` |
| `mem/qos/mem_ctrl.hh` | `//src/mem/qos:mem_ctrl_qos` or `:mem_ctrl_qos_hh` |
| **`sim`** | |
| `sim/clocked_object.hh`| `//src/sim:clocked_object_hh` |
| `sim/sim_object.hh` | `//src/sim:sim_object_hh` |
| `sim/system.hh` | `//src/sim:system_hh` |

### 6.2. Python `m5.objects` to Bazel Target Mapping (`py_library`)

When a Python SimObject file has an `import` statement, the `deps` list of its `py_library` should point to the target that provides that object. The convention is to use a shim target defined in `//src/python/m5/objects/BUILD`.

| `from m5.objects import <ObjectName>` | Bazel Target in `deps` |
| --- | --- |
| `AbstractMemory` | `//src/python/m5/objects:AbstractMemory` |
| `BaseCPU` | `//src/python/m5/objects:BaseCPU` |
| `Cache` | `//src/python/m5/objects:Cache` |
| `ClockedObject` | `//src/python/m5/objects:ClockedObject` |
| `MemCtrl` | `//src/python/m5/objects:MemCtrl` |
| `MemInterface` | `//src/python/m5/objects:MemInterface` |
| `QoSMemCtrl` | `//src/python/m5/objects:QoSMemCtrl` |
| `SimpleMemory` | `//src/python/m5/objects:SimpleMemory` |
| `SimObject` | `//src/python/m5:SimObject` |
| `System` | `//src/python/m5/objects:System` |

### 6.3. Debug Flag to Bazel Target Mapping (`gem5_debug_flag`)

When a C++ file uses `DPRINTF(FlagName, ...)`, the `gem5_cc_library` target must have a dependency on the corresponding `gem5_debug_flag` target.

| `DPRINTF` Flag | Bazel Target in `deps` |
| --- | --- |
| `LLSC` | `//src/mem:debug_LLSC` |
| `MemoryAccess` | `//src/mem:debug_MemoryAccess` |
| `HtmMem` | `//src/mem:debug_HtmMem` |
| `Bridge` | `//src/mem:debug_Bridge` |
| `Cache` | `//src/mem/cache:debug_Cache` |
| `MSHR` | `//src/mem/cache:debug_MSHR` |
| `Commit` | `//src/cpu:debug_Commit` |
| `Fetch` | `//src/cpu:debug_Fetch` |
| `Arm` | `//src/arch/arm:debug_Arm` |

---

## 7. Debugging `gem5_sim_object` Failures

The `gem5_sim_object` macro for a SimObject `MyObject` creates multiple underlying targets. To fully debug failures, you should build these targets in order:

1.  **`//path/to:MyObject_param_gen`**: Tests the Python code generation step.
2.  **`//path/to:param_MyObject_hh`**: Tests the compilation of the generated C++ header.
3.  **`//path/to:param_MyObject_py_pybind`**: Tests the compilation of the generated C++ source and Python bindings.

### 7.1. Python Module Not Found or NameErrors (`extra_tool_deps`)

#### 7.1.1. ModuleNotFoundError


**Symptom:** Failure when building `//path/to:MyObject_param_gen`.

**Error Signature:**

```
ERROR: path/to/BUILD:X:Y: Executing genrule //path/to:MyObject_param_gen failed: ...
Traceback (most recent call last):
  ...
ModuleNotFoundError: No module named 'src.python.m5.objects.MissingObject'

The above exception was the direct cause of the following exception:
  ...
ImportError: Could not import MissingObject from m5.objects
```

This indicates that the Python script run by the generator to parse your SimObject's Python file (e.g., `MyObject.py`) failed because it couldn't find a necessary Python module.

**Root Cause:**

The `gem5_sim_object` rule needs to load and introspect the Python class definition of your SimObject. If your SimObject's parameters involve other SimObject types, the generator's Python environment needs access to those SimObject definitions as well. These dependencies are provided to the generator tool via the `extra_tool_deps` attribute.

The error above means that the `py_library` target for `MissingObject` was not included in the `extra_tool_deps` list of the `gem5_sim_object` rule for `MyObject`.

**Identifying the Missing Dependency:**

There are two primary ways to find out which dependency is missing:

1.  **Analyze the Build Error:** The `ModuleNotFoundError` or `ImportError` message in the build log will usually pinpoint the exact module path that couldn't be found (e.g., `m5.objects.MissingObject`). This tells you that the `MissingObject` SimObject is the missing dependency.

2.  **Inspect the SimObject Python File:** Open the `.py` file for the SimObject whose generator is failing (e.g., `MyObject.py`). Look for parameter definitions like this:
    ```python
    some_param = Param.MissingObject("Description of parameter")
    # or
    other_param = VectorParam.MissingObject("Description of vector parameter")
    ```
    If the type used with `Param.` or `VectorParam.` is not a basic Python type (e.g., `Int`, `String`, `Bool`), an `Enum`, or a standard collection, it's a reference to another SimObject. The name used (e.g., `MissingObject`) is the SimObject type that the generator needs a dependency for.

**The Fix:**

To fix this, you need to add the `py_library` target for the missing SimObject to the `extra_tool_deps` list of the failing `gem5_sim_object` rule in the `BUILD` file.

The `py_library` targets for most base SimObjects are defined in `//src/python/m5/objects/BUILD`. The target name typically matches the SimObject name.

**Example:** If the error is `Could not import QoSPolicy from m5.objects`, you need to add:

```python
# In the gem5_sim_object rule for the SimObject
extra_tool_deps = [
    # ... other deps
    "//src/python/m5/objects:QoSPolicy",
],
```

**Indirect Dependencies:** Sometimes, the missing dependency might not be directly used in *your* SimObject's Python file, but in one of the SimObjects it inherits from. The build error is the most reliable way to find the specific missing module in these cases. You still fix it by adding the missing SimObject's `py_library` to the `extra_tool_deps` of the *failing* generator rule.

**Make sure the SimObject is in `sim_objects.bzl`:** The `m5.objects` shim layer relies on the SimObject being registered in `//bazel/sim_objects.bzl`. If the object is not registered, you'll get a `ModuleNotFoundError` even if the `extra_tool_deps` are correct.

By iteratively adding the missing dependencies reported in the build errors to `extra_tool_deps`, you can resolve these generator failures.

#### 7.1.2. NameError: name 'Enum' or 'Self' is not defined

**Symptom:** Failure when building `//path/to:MyObject_param_gen` or an `enum_generator` target.

**Error Signature:**

```
NameError: name 'Enum' is not defined
# or
NameError: name 'Self' is not defined
```

This typically occurs in a SimObject's Python file (`.py`).

**Root Cause:**

The Python file is missing necessary imports from the gem5 standard library.

*   `Enum`: Base class for gem5 enumerations, defined in `m5.params`.
*   `Self`: Used to refer to other parameters within the same SimObject definition, defined in `m5.proxy`.

**The Fix:**

Ensure the following imports are present at the top of the SimObject's Python file:

```python
from src.python.m5.params import *
from src.python.m5.proxy import *
```

Adding these wildcard imports will bring `Enum`, `Self`, `Param`, and other necessary components into scope.

**BUILD File Dependencies:**

Make sure the `py_library` target for this SimObject file has the corresponding dependencies in its `deps` list:

```python
py_library(
    name = "MyObject_py",
    srcs = ["MyObject.py"],
    deps = [
        # ... other object dependencies
        "//src/python/m5/params",
        "//src/python/m5:proxy",
    ],
)
```

Additionally, any `gem5_enum_generator` or `gem5_sim_object` that uses this `py_library` (e.g., in `enum_py_lib` or `sim_object_py_lib`) must also include these in their `extra_tool_deps`:

```python
extra_tool_deps = [
    # ... other tool dependencies
    "//src/python/m5/params",
    "//src/python/m5:proxy",
],
```

### 7.2. C++ Header Not Found (`extra_param_deps`)

**Symptom:** Failure when building `//path/to:param_MyObject_hh`.

**Error Signature:**

```
ERROR: path/to/BUILD:X:Y: Compiling path/to/params/MyObject.hh failed: ...
bazel-out/k8-fastbuild/genfiles/path/to/params/MyObject.hh:19:10: fatal error: 'params/MissingHeader.hh' file not found
   19 | #include "params/MissingHeader.hh"
      |          ^~~~~~~~~~~~~~~~~~~~~~~
1 error generated.
```

This error means a C++ header file, expected by the code generated by `gem5_sim_object`, was not found. The generated headers often need to include the parameter headers of other SimObjects that are used as parameter types.

**Root Cause:**

The `gem5_sim_object`'s `extra_param_deps` attribute lists the C++ header targets required to compile the generated `params/MyObject.hh` file. This list must include the `*_sim_object` targets for any SimObject types used as parameters within `MyObject.py`.

**The Fix:**

1.  **Identify Missing Header:** The error message clearly states which file was not found (e.g., `params/MissingHeader.hh`). This tells you that the parameter header for the `MissingHeader` SimObject is missing.

2.  **Find the Target:** The required target is usually the name of the missing header target, typically found in the BUILD file where `MissingHeader` is defined. For example, if `MissingHeader.hh` is the C++ implementation for the `MissingHeader` SimObject, the target you need is likely `:MissingHeader_sim_object` or `//path/to/missing:MissingHeader_sim_object`.

3.  **Update `extra_param_deps`:** Add the target for the missing header to the `extra_param_deps` list of the `gem5_sim_object` rule for `MyObject`.

    ```python
    # In the gem5_sim_object rule for MyObject
    extra_param_deps = [
        # ... other deps
        ":MissingHeader_sim_object",
    ],
    ```

4.  **Fix Dependencies of Dependencies:** As seen in the `QoSMemCtrl` example, the missing header might be from a SimObject that *your* SimObject depends on. The build error will still point to the ultimate missing file. You might need to add the missing `extra_param_deps` to the `gem5_sim_object` rule of the *dependency* SimObject. For instance, if `MyObject` uses `OtherObject` as a parameter, and the build fails to find `params/BaseObject.hh`, you might need to add `//path/to:BaseObject_sim_object` to the `extra_param_deps` of the `gem5_sim_object` rule for `OtherObject`.

By iteratively building, checking the error, and adding the missing `*_sim_object` targets to `extra_param_deps` in the appropriate generator rule, you can resolve these header-not-found errors.

### 7.3. C++ Compilation/Linking Errors (`extra_deps` and standard `deps`)

**Symptom:** Failure when building `//path/to:param_MyObject_py_pybind` or runtime `ImportError` / `dlopen` failures for `_m5_param_MyObject.so`.

**Error Signature:**

These can be varied, including:

*   `error: module //path/to:param_MyObject_py_pybind does not depend on a module exporting 'some/header.hh'`
*   `undefined symbol: SomeClass::some_function()`
*   `ImportError: dlopen(.../_m5_param_System.so, X): symbol not found in flat namespace '__ZN4gem55enums...'`

**Root Cause:**

These errors occur because the *generated C++ source file* (e.g., `bazel-out/.../genfiles/path/to/param_MyObject.cc`) has missing dependencies, or because the underlying `gem5_cc_library` targets used by the SimObject implementation are missing dependencies.

*   **`extra_deps`:** The `gem5_sim_object` uses the `extra_deps` argument to add dependencies to the `cc_library` that compiles the generated `param_MyObject.cc` file. This is where you link against the C++ libraries that define the types and functions used in the parameter wrappers.
*   **Standard `deps`:** Errors can also surface from missing dependencies in the core `gem5_cc_library` targets for the SimObject itself (e.g., `:my_object`) or any of its dependencies.

**The Fix:**

1.  **Analyze the Error:** The error message will typically point to a missing header or an undefined symbol.
2.  **Identify the Missing Library:** Determine which `gem5_cc_library` target provides the missing header or symbol. This might be the SimObject's own implementation library, or a utility library.
3.  **Update Dependencies:**
    *   If the generated `param_MyObject.cc` file is missing a dependency, add the required `gem5_cc_library` target to the `extra_deps` list of the `gem5_sim_object` rule.
    *   If the error is within a standard `gem5_cc_library` (e.g., `:my_object`), add the missing dependency to the `deps` list of *that* rule. This might involve fixing targets in different BUILD files.
4.  **Header/Implementation Splits:** Remember to depend on the correct split target. For headers, use the `_hh` target if available. For linking, use the implementation target.

### 7.4. Avoid Self-Dependencies in `extra_tool_deps`

**Rule:** Never depend on `m5/objects:<filename>` in `extra_tool_deps` if the `sim_object_module`'s filename is `<filename>`.

**Explanation:**
When defining a `gem5_sim_object`, the `sim_object_py_lib` attribute points to the `py_library` containing the SimObject's definition (e.g., `:MyObject_py`). This library is implicitly available to the generator tool.

Adding `//src/python/m5/objects:MyObject` to `extra_tool_deps` essentially adds the same dependency a second time, but via the `m5.objects` shim. This redundant self-dependency can confuse the build system or the generator tool and is considered a configuration error. This type of configuration error will often manifest as a circular dependency in the build failure log, so if you encounter such an error, verify that this rule is being enforced.

**Example of Incorrect Configuration:**

**Note:** You can identify this issue if you see an error like `ImportError: cannot import name 'X' from partially initialized module 'Y' (most likely due to a circular import)`.

```python
gem5_sim_object(
    sim_object_name = "MyObject",
    sim_object_module = "path.to.MyObject", # Module is MyObject
    sim_object_py_lib = ":MyObject_py",
    extra_tool_deps = [
        # INCORRECT: Do not depend on MyObject here!
        "//src/python/m5/objects:MyObject",
    ],
)
```

### 7.5. Recommended Debugging Order

When a `gem5_sim_object` for `MyObject` is failing, build the related targets in this order:

1.  **`//path/to:MyObject_param_gen`**: Catches Python errors. Check `extra_tool_deps` and `sim_objects.bzl`.
2.  **`//path/to:param_MyObject_hh`**: Catches missing C++ param headers. Check `extra_param_deps`. Use the `*_sim_object` target to refer to other SimObjects.
3.  **`//path/to:param_MyObject_py_pybind`**: Catches C++ code issues. Check `extra_deps` and standard `deps`.

## 7. Understanding the gem5 Bazel Build System

### 7.1. Core Components

*   **`BUILD` files:** Standard Bazel files declaring targets using custom gem5 rules.
*   **`//bazel/rules.bzl`:** Defines the custom Starlark rules like `gem5_cc_library`, `gem5_sim_object`, etc.
*   **`//bazel/sim_objects.bzl`:** This is a CRITICAL file. It acts as a central registry for all SimObjects. Every SimObject class that needs to be accessible from Python configuration files or other SimObjects must have an entry in the `SIM_OBJECTS` list.
*   **SimObject Python Files (e.g., `src/mem/MyMem.py`):** These files contain the Python class definitions for SimObjects, inheriting from `SimObject` and using `Param` for parameters.
*   **SimObject C++ Files (e.g., `src/mem/my_mem.hh`, `src/mem/my_mem.cc`):** These contain the C++ implementation.

### 7.2. The `m5.objects` Shim Layer

When you write `from m5.objects import MySimObject` in a Python script, you are not directly importing the source file (e.g., `src/mem/MySimObject.py`). Instead, you are interacting with a shim layer:

1.  **`//src/python/m5/objects/__init__.py`:** This file acts as a gatekeeper. When an import like `from m5.objects import X` is encountered, this `__init__.py` looks up `X` in the `SIM_OBJECTS` list from `sim_objects.bzl`.
2.  **Dynamic Import:** Based on the `module_path` and `dep` in `sim_objects.bzl`, the `__init__.py` dynamically imports the actual Python module containing the SimObject definition.

This mechanism allows gem5 to know about all SimObjects without needing to know the exact file path for each at the point of import.

### 7.3. How `gem5_sim_object` Works

The generator macro for `MyObject` does the following:

1.  Runs a Python tool (e.g., `create_param_cc.py`).
2.  This tool **imports** the SimObject's Python module (e.g., `src.mem.MyObject`) to introspect its class definition, parameters, and types. This import goes through the `m5.objects` shim.
3.  It generates C++ header (`params/MyObject.hh`) and source (`param_MyObject.cc`) files that define the parameter struct and pybind wrappers.

### 7.4. Why Cyclic Dependencies Happen in Bazel

Bazel requires a strict Directed Acyclic Graph (DAG) for dependencies, which is analyzed *before* build execution. Cycles can occur due to:

1.  **Python Import Cycles:** SimObject A's Python file imports SimObject B from `m5.objects`, and SimObject B's Python file imports SimObject A from `m5.objects`. When the generator tries to load one, the `m5.objects` shim tries to load the other, leading to a `ModuleNotFoundError` because neither module has finished loading. This is the type of cycle we saw with `QoSMemSinkCtrl` and `QoSMemSinkInterface`.

2.  **C++ Header Cycles:** C++ header files from different targets `#include` each other in a way that forms a loop. This is often solvable by splitting `gem5_cc_library` targets into `_hh` and implementation targets, and using forward declarations.

**SCons vs. Bazel:** SCons is more dynamic. It doesn't perform the same upfront static dependency analysis. Its Python execution model during the build might also be more lenient with import ordering, masking potential cyclic issues that Bazel catches.

## 8. Handling Circular Dependencies

*   **Python Cycles:** Refactor by moving the shared component (often an Enum) to a more fundamental module, or use `if typing.TYPE_CHECKING:` blocks for imports. Update `BUILD` files and `sim_objects.bzl` accordingly.
*   **C++ Cycles:** Utilize the header (`_hh`) / implementation split. Depend on `_hh` targets whenever possible. Forward declare classes in headers if full definitions are not needed.

### 8.1. Case Study: Fixing Cycle Between mem_interface.hh and mem_ctrl.hh

A real-world example of a cyclic dependency was found in `//src/mem/BUILD` between the `:mem_interface_hh` and `:mem_ctrl_hh` targets.

**1. The Problem:**

Bazel reported a dependency cycle:
`:mem_interface_hh` -> `:mem_ctrl_hh` -> `:mem_interface_hh`

**2. Analysis:**

*   `mem_interface.hh` included `mem/mem_ctrl.hh`. This was necessary because `MemInterface` used types defined in `mem_ctrl.hh`, namely `MemPacket`, `MemPacketQueue`, and also needed access to `MemCtrl::BusState`.
*   `mem_ctrl.hh` included `mem/qos/mem_ctrl.hh` (which defines `qos::MemCtrl::BusState`) and also contained a member `MemInterface* dram;`, requiring the definition of `MemInterface`.

The cycle arose because each header needed definitions from the other.

**3. Refactoring Strategy:**

To break the cycle, the type definitions required by both headers had to be accessible without a circular include. The types `BurstHelper`, `MemPacket`, and `MemPacketQueue` were moved from `mem_ctrl.hh` to `mem_interface.hh`.

**4. Source File Changes:**

*   **`mem_interface.hh`:**
    *   The definitions of `BurstHelper`, `MemPacket`, and `MemPacketQueue` were moved into this file, before the `MemInterface` class definition.
    *   `#include "mem/mem_ctrl.hh"` was removed.
    *   `#include "mem/qos/mem_ctrl.hh"` was added to gain access to `qos::MemCtrl::BusState`.
    *   A forward declaration `class MemCtrl;` was added for the `gem5::memory::MemCtrl`.
    *   Usages of `MemCtrl::BusState` were updated to `qos::MemCtrl::BusState`.

*   **`mem_ctrl.hh`:**
    *   The definitions of `BurstHelper`, `MemPacket`, and `MemPacketQueue` were removed.
    *   `#include "mem/mem_interface.hh"` was added to access the moved definitions.

**5. BUILD File Changes (`//src/mem/BUILD`):**

*   Target `:mem_interface_hh`:
    *   Removed `":mem_ctrl_hh"` from `deps`.
    *   Added `"//src/mem/qos:mem_ctrl_qos_hh"` to `deps`.

*   Target `:mem_ctrl_hh`:
    *   Added `":mem_interface_hh"` to `deps`.
    *   Added `":qport"` to `deps`.

*   Target `:mem_interface`:
    *   Added `"//src/mem/qos:mem_ctrl_qos_hh"` to `deps`.

*   Target `:mem_ctrl`:
    *   Added missing dependencies like `":enum_MemSched_hh"`, `":param_MemCtrl_hh"`, `"//src/base:callback"`, etc.

*   Target `:NVMInterface_py`:
    *   Corrected an `ImportError` by changing an import from `m5.objects.DRAMInterface` to `m5.objects.MemInterface` for `AddrMap` in `NVMInterface.py`.

*   Target `gem5_sim_object` for `MemCtrl`:
    *   Added `":mem_interface_hh"` to `extra_deps`.

**6. Subsequent Fixes:**

*   `dram_interface.cc`: Added `#include "mem/mem_ctrl.hh"` and updated `:dram_interface` deps to include `:mem_ctrl_hh` because it uses `MemCtrl` members.

This refactoring successfully broke the dependency cycle, allowing all targets in `//src/mem/BUILD` to build.

## 9. Coding Style Guidelines

The gem5 codebase adheres to a specific coding style that must be maintained.

### 9.1. Formatting Rules

*   **Indentation:**
    *   4 spaces per level.
    *   Use spaces only (no tabs).
    *   Namespaces should **not** increase the indentation.
    *   Exception: Case labels, `public:`, `private:`, `protected:` modifiers, and goto labels are indented 2 spaces.
*   **Line Length:** Maximum 79 characters.
*   **Braces:**
    *   **Control Blocks (if, while, etc.):** Opening brace on the same line as the keyword, separated by a space.
        ```cpp
        if (...) {
            ...
        }
        ```
    *   **Functions & Classes:** Opening brace on a *new line* (Allman style), in the first column.
        ```cpp
        int
        exampleFunc(...)
        {
            ...
        }
        ```
    *   **Else/Else If:** Keyword follows the closing brace on the same line.
        ```cpp
        } else {
        ```
    *   **Single Statement Blocks:** Braces are optional if the statement fits on one line and isn't part of a complex chain. If any block in an if/else chain needs braces, *all* blocks should have them.
*   **Spacing:**
    *   One space between keywords and opening parentheses (`if (...)`).
    *   One space around binary operators (`a + b`, `a = b`).
    *   No space around `=` in parameter lists for default values.
    *   No space between function name and arguments (`func(arg)`).
    *   No space immediately inside parentheses `(arg)`, unless complex.
*   **Naming:**
    *   **Classes/Types:** `MixedCase` (e.g., `ClassName`). Acronyms all uppercase (e.g., `CPU`).
    *   **Members:** `mixedCase` (e.g., `aMemberVariable`).
    *   **Accessors:** Members with accessors start with `_` (e.g., `_foo`). Accessor function matches variable name without `_` (e.g., `foo()`).
    *   **Locals/Parameters:** `snake_case` (e.g., `local_variable`).
    *   **Macros/Constants:** `ALL_CAPS`.
*   **Includes:**
    *   Favor C++ headers (`<cstdio>`) over C headers (`<stdio.h>`).
    *   Order: `Python.h` (if needed), Main Header, C includes, C++ includes, gem5 shared includes, M5 includes. Sorted alphabetically within groups.

### 9.2. Refactoring Rules
*   **Code Changes Only:** When refactoring code (e.g., breaking circular dependencies, moving function implementations to source files), ensure that you **only** modify the code structure (e.g., moving functions, adding includes, changing declarations).
*   **Preserve Formatting:** Do **not** apply any auto-formatting or style changes to the surrounding code.
*   **Consistency:** Ensure that any new code added follows the existing style of the file (4 spaces indentation, Allman braces for functions/classes).

**Example:**
If you move a function implementation from a `.hh` file to a `.cc` file:
1.  Copy the function body to the `.cc` file.
2.  Ensure the indentation in the `.cc` file matches the 4-space rule.
3.  Ensure the braces for the function definition are placed on new lines.
4.  Do not reformat the rest of the `.hh` or `.cc` file.

## 9. Strict Dependency Checking

Strict dependency checking ensures that all imports in your Python code are explicitly declared as dependencies in your `BUILD` file. This can cause issues with gem5's `m5.objects` dynamic import system, especially for "other" SimObjects (Python-only subclasses).

### 9.1. The Problem

You might encounter an error like this:

```
ERROR: Strict deps violations: //path/to:your_target
  In path/to/your_script.py, no direct deps found for imports:
    from src.python.m5.objects import MyOtherSimObject: Ambiguous import...
```

This happens because `m5.objects` dynamically loads `MyOtherSimObject` at runtime. The static analysis tool (strict deps checker) looks at `m5.objects` and sees that it does not explicitly export `MyOtherSimObject`.

### 9.2. The Solution

Instead of importing from the generic `m5.objects` package, import the SimObject from its specific parent module where it is re-exported.

1.  **Identify the Parent SimObject:** Check `bazel/sim_objects.bzl` to find which entry contains your SimObject in its `other_simobjects` list.

    ```python
    {
        "mod_name": "ParentSystem",
        "module_path": "...",
        "dep": "...",
        "simobjects": ["ParentSystem"],
        "other_simobjects": ["MyOtherSimObject"],
    }
    ```

    In this example, `MyOtherSimObject` belongs to the `ParentSystem` module.

2.  **Update Python Imports:** Change your import statement to point to the specific module.

    **Old (Failing):**
    ```python
    from m5.objects import MyOtherSimObject
    ```

    **New (Fixed):**
    ```python
    from src.python.m5.objects.ParentSystem import MyOtherSimObject
    ```

3.  **Update BUILD Dependencies:** Ensure your target depends on the parent SimObject's library.

    ```python
    deps = [
        "//src/python/m5/objects:ParentSystem",
    ]
    ```

This works because the `m5.objects.ParentSystem` module is generated to explicitly export both its main `simobjects` and its `other_simobjects`, satisfying the strict dependency checker.

## 10. Advanced Troubleshooting and Common Pitfalls

### 10.1. Python Module Identity and `isinstance` Failures

**Problem:** In the Bazel environment, Python modules may be imported multiple times through different paths (e.g., as part of a tool's `runfiles` and via the `m5.objects` shim). This causes `isinstance(obj, Class)` to fail even if `obj` is logically an instance of `Class`, because the two `Class` definitions come from different module instances.

**Symptoms:**
- `AttributeError` or `SimObject` initialization errors where expected parameters are missing.
- `Proxy` resolution failures because `isSimObject(obj)` returns `False`.

**Solution: Duck-Typing**
Avoid using `isinstance()` for core gem5 classes (`SimObject`, `ParamValue`, `Proxy`, etc.). Instead, use identifying attributes:
1. Ensure the base classes have a unique identifying attribute (e.g., `SimObject` has `_is_simobject = True`).
2. Use `getattr(obj, '_is_simobject', False)` or similar "duck-typing" checks instead of `isinstance`.

### 10.2. MetaSimObject Initialization and `_init_called`

**Problem:** `MetaSimObject.__init__` uses a `_init_called` attribute to prevent double initialization. However, because this attribute is often inherited, a subclass (like `SimpleObject`) might see `_init_called = True` from its parent (`SimObject`), causing the subclass to skip its own initialization (e.g., setting up `_base`, `_params`, etc.).

**Symptom:** Subclasses of `SimObject` have `_base = None` or missing `_params` even though they are clearly SimObjects.

**Solution: Class-Local Check**
Check for `_init_called` specifically in the class's own `__dict__` to ensure it's not inherited:
```python
if cls.__dict__.get("_init_called", False):
    return
```

### 10.3. Root SimObject Identification in Generators

**Problem:** Parameter generation scripts (e.g., `sim_object_param_struct_hh.py`) may use identity comparison (`sim_object == SimObject`) to identify the root `SimObject` class (to add special parameters like `name`). This fails if `SimObject` has been imported via different paths.

**Solution: Path/Type Comparison**
Use more robust checks, such as checking the `.type` attribute or the module path:
```python
if sim_object.type == "SimObject" and not sim_object._base:
    # This is the root SimObject
```

### 10.4. macOS `std::filesystem` Compatibility

**Problem:** Compiling C++17 code using `std::filesystem` on macOS may fail if the deployment target is set too low (the default for some Bazel configurations).

**Symptoms:**
- `error: 'path' is unavailable: introduced in macOS 10.15`
- `error: 'current_path' is unavailable: introduced in macOS 10.15`

**Solution: `.bazelrc` Configuration**
Set the minimum macOS version in your `.bazelrc`:
```bash
build --copt="-mmacosx-version-min=10.15"
build --linkopt="-mmacosx-version-min=10.15"
```
Also ensure `--cxxopt="-std=c++17"` is set.
