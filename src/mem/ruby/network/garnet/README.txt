README for Garnet3.0
Written By: Tushar Krishna (tushar@ece.gatech.edu)
Last Updated: Sep 9, 2020
-------------------------------------------------------

Garnet Network Parameters and Setup:
- GarnetNetwork.py
    * defaults can be overwritten from command line (see configs/network/Network.py)
- GarnetNetwork.hh/cc
    * sets up the routers and links
    * collects stats


CODE FLOW
- NetworkInterface.cc::wakeup()
    * Every NI connected to one coherence protocol controller on one end, and one router on the other.
    * receives messages from coherence protocol buffer in appropriate vnet and converts them into network packets and sends them into the network.
        * garnet adds the ability to capture a network trace at this point.
    * receives flits from the network, extracts the protocol message and sends it to the coherence protocol buffer in appropriate vnet.
    * manages flow-control (i.e., credits) with its attached router.
    * The consuming flit/credit output link of the NI is put in the global event queue with a timestamp set to next cycle.
      The eventqueue calls the wakeup function in the consumer.

- NetworkLink.cc::wakeup()
    * receives flits from NI/router and sends it to NI/router after m_latency cycles delay
        * Default latency value for every link can be set from command line (see configs/network/Network.py)
        * Per link latency can be overwritten in the topology file
    * The consumer of the link (NI/router) is put in the global event queue with a timestamp set after m_latency cycles.
      The eventqueue calls the wakeup function in the consumer.

- Router.cc::wakeup()
    * Loop through all InputUnits and call their wakeup()
    * Loop through all OutputUnits and call their wakeup()
    * Call SwitchAllocator's wakeup()
    * Call CrossbarSwitch's wakeup()
    * The router's wakeup function is called whenever any of its modules (InputUnit, OutputUnit, SwitchAllocator, CrossbarSwitch) have
      a ready flit/credit to act upon this cycle.

- InputUnit.cc::wakeup()
    * Read input flit from upstream router if it is ready for this cycle
    * For HEAD/HEAD_TAIL flits, perform route computation, and update route in the VC.
    * Buffer the flit for (m_latency - 1) cycles and mark it valid for SwitchAllocation starting that cycle.
        * Default latency for every router can be set from command line (see configs/network/Network.py)
        * Per router latency (i.e., num pipeline stages) can be set in the topology file

- OutputUnit.cc::wakeup()
    * Read input credit from downstream router if it is ready for this cycle
    * Increment the credit in the appropriate output VC state.
    * Mark output VC as free if the credit carries is_free_signal as true

- SwitchAllocator.cc::wakeup()
    * Note: SwitchAllocator performs VC arbitration and selection within it.
    * SA-I (or SA-i): Loop through all input VCs at every input port, and select one in a round robin manner.
        * For HEAD/HEAD_TAIL flits only select an input VC whose output port has at least one free output VC.
        * For BODY/TAIL flits, only select an input VC that has credits in its output VC.
    * Place a request for the output port from this VC.
    * SA-II (or SA-o): Loop through all output ports, and select one input VC (that placed a request during SA-I) as the winner for this output port in a round robin manner.
        * For HEAD/HEAD_TAIL flits, perform outvc allocation (i.e., select a free VC from the output port).
        * For BODY/TAIL flits, decrement a credit in the output vc.
    * Read the flit out from the input VC, and send it to the CrossbarSwitch
    * Send a increment_credit signal to the upstream router for this input VC.
        * for HEAD_TAIL/TAIL flits, mark is_free_signal as true in the credit.
        * The input unit sends the credit out on the credit link to the upstream router.
    * Reschedule the Router to wakeup next cycle for any flits ready for SA next cycle.

- CrossbarSwitch.cc::wakeup()
    * Loop through all input ports, and send the winning flit out of its output port onto the output link.
    * The consuming flit output link of the router is put in the global event queue with a timestamp set to next cycle.
      The eventqueue calls the wakeup function in the consumer.


If a clock domain crossing(CDC) or Serializer-Deserializer unit is
instantiated, then the Network Brisge takes over the flit in HeteroGarnet.
- NetworkBridge::wakeup()
    * Check if SerDes is enabled and do appropriate calculations for
    serializing or deserializing the flits
    * Check if CDC is enabled and schedule all the flits according
    to the consumers clock domain.
