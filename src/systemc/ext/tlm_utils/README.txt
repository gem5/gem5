
TLM-2.0 standard utilities
==========================

Dir: include/tlm_utils

SubDirs:

Files: README.txt
       instance_specific_extensions.h
       multi_passthrough_initiator_socket.h
       multi_passthrough_target_socket.h
       multi_socket_bases.h
       peq_with_get.h
       simple_initiator_socket.h
       simple_target_socket.h
       peq_with_cb_and_phase.h
       passthrough_target_socket.h
       tlm_quantumkeeper.h


Comments
========

This directory contains a number of ease-of-use and convenience implementations
for the interoperability standard. All objects defined in the header files of
this directory are contained in the tlm_util namespace.
There is no tlm_utils.h header files containing all includes in order to avoid
additional dependencies for TLM 2.0

Files:
  simple_initiator_socket.h
     version of an initiator socket that has a default implementation of all
     interfaces and allows to register an implementation for any of the
     interfaces to the socket, either unique interfaces or tagged interfaces
     (carrying an additional id)

  simple_target_socket.h
     version of a target socket that has a default implementation of all
     interfaces and allows to register an implementation for any of the
     interfaces to the socket, either unique interfaces or tagged interfaces
     (carrying an additional id)
     This socket allows to register only 1 of the transport interfaces
     (blocking or non-blocking) and implements a conversion in case the
     socket is used on the other interface

  passthrough_target_socket.h
     version of a target socket that has a default implementation of all
     interfaces and allows to register an implementation for any of the
     interfaces to the socket.

  multi_passthrough_initiator_socket.h
     an implementation of a socket that allows to bind multiple targets to the
     same initiator socket. Implements a mechanism to allow to identify in the
     backward path through which index of the socket the call passed through

  multi_passthrough_target_socket.h
     an implementation of a socket that allows to bind multiple initiators to
     the same target socket. Implements a mechanism to allow to identify in the
     forward path through which index of the socket the call passed through

  multi_socket_bases.h
     contains base class definitions used by the multi_passthrough sockets

  peq_with_get.h
     payload event queue (PEQ) implementation using a pull interface.
     Has a get_next_transaction API that returns the transaction that is
     scheduled in the event queue

  peq_with_cb_and_phase.h
     another payload event queue, this one with a push interface (callback
     mechanism ). Allows to register a callback that will be called whenever
     the event in the event queue is triggered, the callback gets transaction
     and phase as arguments

  instance_specific_extensions.h
     is an implementation for adding extentions in the generic payload that
     are specific to an instance along the path of a transaction, to allow that
     extentions of the same type can be used by the different blocks along
     the path of the transaction

  tlm_quantumkeeper.h
     is an convenience object used to keep track of the local time in
     an initiator (how much it has run ahead of the SystemC time), to
     synchronize with SystemC time etc.
