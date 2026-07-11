# Ruby Agent Notes

Ruby combines hand-written C++/Python, SCons glue, and SLICC protocol inputs.
Generated protocol output belongs under `build/`; do not edit it directly.

## SLICC And Generated Files

Protocol source files live under `src/mem/ruby/protocol`. If generated C++ or
HTML looks wrong, change the SLICC input or generator path, then rebuild. Keep
SCons emitters side-effect free: emitters should discover targets, while
actions should write generated files.

## Ruby And Classic

gem5 has two main cache-hierarchy families. Classic caches model a relatively
direct cache/memory object graph in C++ and are usually configured by composing
cache, bus, and memory objects. Ruby models cache-coherence protocols as
message-passing controller state machines connected through Ruby networks.

Ruby protocols are written in SLICC under `src/mem/ruby/protocol`; SLICC
generates controller C++ code, protocol message types, and HTML protocol
documentation under `build/`. Protocol behavior should be changed in the SLICC
state machines or the Ruby support code they call, not in generated files.

When working on coherence behavior, identify whether the bug belongs to the
Ruby protocol, the generated controller, the network, the Sequencer, or the
memory-system interface. For Classic behavior, start with the concrete cache or
interconnect object instead. Do not assume a Classic fix applies to Ruby or
that a Ruby protocol fix applies to every protocol.

## Review Notes

For protocol path or case changes, verify generated paths and source directory
case exactly.
