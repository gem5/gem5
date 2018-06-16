SimpleLTInitiator1/SimpleLTTarget1
----------------------------------

- LT Initiator/Target model using the base (standard) tlm socket
- Added support for DMI in SimpleLTTarget1

SimpleLTInitiator1_DMI
----------------------

- uses DMI transactions, the DMI structure is using the DMI-hint
  to check if a DMI request would make sense.
- uses a single transport_dbg transaction at end_of_simulation()

SimpleLTInitiator2/SimpleLTTarget2
----------------------------------

- LT Initiator/Target model using the convenience tlm socket
- Target and Initiator model use the REGISTER_DEBUGTRANSPORT macro to register
  a transport callback to the socket
- Added support for DMI handling, callback registration with
  REGISTER_DMI
- SimpleLTTarget2 does not register the transport_dbg callback, so that
  we are able to test this case in bus_dmi.

SimpleLTInitiator2_DMI
----------------------

- uses DMI transactions, but ignoring the DMI hint
- uses a single transport_dbg transaction at end_of_simulation()

SimpleLTInitiator3
------------------

- LT Initiator model using the convenience tlm socket
- Initiator model uses the endEvent of the socket to wait until the
  transaction is finished

SimpleLTInitiator3_DMI
----------------------

- based on SimpleInitiator3, uses DMI (without DMI hint)

SimpleATInitiator1/SimpleATTarget1
----------------------------------

- AT Initiator/Target model implementing the AT protocol
- one call of nb_transport for each timing point in the protocol (BEGIN_REQ,
  END_REQ, BEGIN_RESP and END_RESP)

SimpleATInitiator2/SimpleATTarget2
----------------------------------

- AT Initiator/Target model implementing the AT protocol with timing annotation
- only a call of nb_transport for the start of a phase (BEGIN_REQ and
  BEGIN_RESP)
- end of a phase is notified via timing annotation (t argument)

CoreDecouplingLTInitiator
-------------------------

- LT Initiator using 'Core Decoupling'

ExplicitLTTarget
----------------

- LT Target that uses explicit timing (calls wait)
- added support for debug transactions

ExplicitLTTarget
----------------

- AT Target, only registers nb_transport

SimpleBus
---------

- Simple bus model
- Runtime switcheable between LT and AT (can only switch if no transactions
  are pending)
- No limitation on number of pending transactions (all targets that can return
  false must support multiple transactions)
- added support for DMI and debug transactions
- LT mode:
-- Forward nb_transport calls to initiator/targets
-- Only one active request/response phase
- AT mode:
-- Incoming transactions are queued
-- AT protocol is executed from a different SC_THREAD
-- A target is notified immediately of the end of a transaction (using timing
   annotation). This is needed because the initiator can re-use the
   transaction (and the target may use the transaction pointer to identify the
   transaction)
