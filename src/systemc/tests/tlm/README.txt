
TLM-2.0 unit tests
==================

Dir: tests/tlm/
SubDirs:
         bus/
         bus_dmi/
         cancel_all/
         common/
         endian_conv/
         multi_sockets/
         nb2b_adapter/
         p2p/
         static_extensions/
         update_original/

Files: README.txt


Comments
========

Running the tests:
------------------

See the main README and README_windows.txt files.


The tests:
----------

All tests are build using a set of models that can be found in the subdirectory
common/include/models. For a description of the models see the README.txt in 
that subdirectory.

The test themselves can be found in the following subdirectories:
bus/ : 
      test system using a TLM2 compliant AT bus model and a combination of
      LT and AT targets and initiators, with and without temporal decoupling. 
      
bus_dmi/ : 
      test system using a similar system as in the bus/ test but now with DMI
      support added for the LT initiators and targets.
      
cancel_all/ :
      Tests the cancel_all() methods of the two PEQs, peq_with_cb_and_phase and peq_with_get.
      Along the way, it also tests the basic operation of each PEQ.
    
endian_conv/ :
      unit test for the endianness conversion ftions, there is only a build for
      linux provided, the C++ test performs a single conversion, there is a 
      python script using the program to do a more extensive test     
      
multi_sockets/ :
      test system using a TLM2 compliant AT busmodel using a single socket to
      bind all targets and initiators to, in combination with LT and AT
      initiators and targets. This test also uses instance specific extentions
      in the generic payload
      
nb2b_adapter/ :
      Primarily a regression test for a bug in the nb2b adapter of the simple_target_socket.
      Tests the operation of the nb2b adapter, and also exercises the peq_with_cb_and_phase
      and the instance_specific_extension.
      
p2p/ :
  BaseSocketLT/ :
  	simple point to point test for LT initiator and target
  CoreDecoupling/
  	simple test for using the LT initiator with temporal decoupling
  EndEventLT/
  	??
  HierarchicalSocket/
    	tests sockets on hierachical modules
  RegisterSocketProcessLT/
  	simple test for initiator and target using callback registry in sockets
  SimpleAT/
  	simple point to point test for AT initiator and target using GP
	AT phases and TLM_ACCEPTED
  SimpleAT_TA/
  	simple point to point test for AT initiator and target using GP
	AT phases and TLM_UPDATED

static_extentions/ :
	contains 3 unit tests to verify the GP extention mechanism

update_original/ :
      Tests the deep_copy_from() and update_original_from() methods of tlm_generic_payload. 
      Along the way, it also tests the use of byte enables for read and write commands.
