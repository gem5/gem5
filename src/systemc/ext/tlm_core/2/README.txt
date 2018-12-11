TLM-2.0 interoperability layer header files
===========================================

Dir: include/tlm_core/2/

SubDirs: interfaces/
	 generic_payload/
	 quantum/
	 sockets

Files: README.txt
       version.h


Comments
========

User code should only #include the tlm or tlm.h header file in the include/
directory and avoid including any of the include files in this directory
directly. All objects defined in this file hierarchy are in the tlm namespace.

tlm_version.h contains the definitions for the version string and integer values

The header files are organizated, by subdirectory, as follows:


interfaces/
-----------------

Contains the TLM-2.0 core interfaces

Files:
      interfaces.h (includes the other header files in this directory )
      fw_bw_ifs.h        (defines the TLM 2.0 interface API's:
					tlm_fw_nonblocking_transport_if
					tlm_bw_nonblocking_transport_if
					tlm_blocking_transport_if
					tlm_fw_direct_mem_if
					tlm_bw_direct_mem_if
					tlm_transport_dbg_if
			  the enumeration type
			   		tlm_sync_enum
			  and the TLM 2.0 standard interfaces using the API's
			   		tlm_fw_transport_if
					tlm_bw_transport_if )
      dmi.h              (defines tlm_dmi)


generic_payload/
--------------------

Contains the TLM-2.0 generic payload and associated classes and helper functions

Files:
      generic_payload.h ( includes the other header files in this directory)
      gp.h                  (defines the TLM 2.0 generic payload classes:
      					tlm_generic_payload
					tlm_extension
					tlm_extension_base
					tlm_mm_interface
			     and the enumeration types
			    		tlm_command
					tlm_response_status  ) 
      array.h               (defines array class used by the extention 
      			     mechanism )
      endian_conv.h         (defines the implementation for the endianness 
      			     helper functions:
			    		tlm_to_hostendian_generic()
					tlm_from_hostendian_generic()
					tlm_to_hostendian_word()
					tlm_from_hostendian_word()
					tlm_to_hostendian_aligned()
					tlm_from_hostendian_aligned()
					tlm_to_hostendian_single()
					tlm_from_hostendian_single()  )
					  
      helpers.h             (defines the helper functions to determine the
      			     hostendianness:
					get_host_endianness()
					host_has_little_endianness()
					has_host_endianness()
			     and defines the enumeration type:
			     		tlm_endianness	
      phase.h               (defines tlm_phase as an extendable enum type)


sockets/
------------

Contains the standard TLM-2.0 initiator and target sockets (which are used as
the base classes for the convenience sockets in tlm_utils)

Files:
      sockets.h              (includes the other header files in this directory)
      initiator_socket.h     (defines the initiator sockets:
					tlm_initiator_socket_base
					tlm_initiator_socket_b
					tlm_initiator_socket
      target_socket.h        (defines the target sockets:
					tlm_target_socket_base
					tlm_target_socket_b
					tlm_target_socket


quantum/
------------
This contains the global quantum. (The quantum keeper is in tlm_utils)

Files:
  quantum.h            ( includes the other header file in this directory )
  global_quantum.h     ( defines tlm_global_quantum ) 
