# The gem5 Components Library

**IMPORTANT NOTE:** This is a Work-In-Process Documentation. This will be expanded and completed in later revisions of the components library.

This is a high-level overview of what this library is.

## Philosophy

Like the [Zen of Python](https://www.python.org/dev/peps/pep-0020/), the gem5 Components Library has a set of guiding principles.
Note, these are note rules, and they are meant to be *bent* if needed (but maybe not broken).

### Components are extensible, not configurable

We prefer *extensibility* instead of *configurability*.
Instead of each component taking many different parameters, we have decided to make many different components.
For instance, instead of having one core component which takes a parameter of the type (e.g., in-order or out-of-order), we specify multiple different components, an `InOrderCPU` and an `OutOfOrder` CPU.

### Components use easy to remember names

We prefer longer and easier to remember names than shorter or jargon names.

## Structure of the components library

### Boards

### Processors

### Memories

### Cache hierarchies

## Contributing to the components library

### Code style

- Use [Black](https://black.readthedocs.io/en/stable/) to format your code.
- Docstring should follow the [ReST style and Sphinx](https://www.sphinx-doc.org/)
