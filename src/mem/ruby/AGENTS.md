# Ruby Agent Notes

Ruby combines hand-written C++/Python, SCons glue, and protocol inputs written
in SLICC, a domain-specific language for describing cache-coherence state
machines. Generated protocol output belongs under `build/`; do not edit it
directly.

## SLICC And Generated Files

Built-in protocol source files live under `src/mem/ruby/protocol`; build logic
can add other protocol directories. If generated C++ or HTML looks wrong,
change the SLICC input or generator path, then rebuild. Keep SCons emitters
side-effect free: emitters should discover targets, while actions should write
generated files.

## Ruby And Classic

gem5 has two main cache-hierarchy families. Classic caches model a relatively
direct cache/memory object graph in C++ and are usually configured by composing
cache, bus, and memory objects. Ruby models cache-coherence protocols as
message-passing controller state machines connected through Ruby networks.

Built-in Ruby protocols are written in SLICC under `src/mem/ruby/protocol`;
additional protocol directories can be supplied by the build. SLICC generates
controller C++ code and protocol types under the selected build directory. It
generates HTML there only when the `SLICC_HTML` Kconfig option is enabled.
Protocol behavior should be changed in the SLICC state machines or the Ruby
support code they call, not in generated files.

When working on coherence behavior, identify whether the bug belongs to the
Ruby protocol, the generated controller, the network, the Sequencer, or the
memory-system interface. For Classic behavior, start with the concrete cache or
interconnect object instead. Do not assume a Classic fix applies to Ruby or
that a Ruby protocol fix applies to every protocol.

## Review Notes

For protocol path or case changes, verify generated paths and source directory
case exactly.

## Validation

Build a configuration that enables the affected Ruby protocol so SLICC runs
and its generated C++ compiles. Then run a focused Ruby test for the changed
state-machine path; successful generation alone does not validate coherence
transitions or message ordering.
