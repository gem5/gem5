/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  pr-503-neg.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

#include "systemc.h"

struct EXC_CTRL : sc_module {
   
    // Input ports
    sc_in<bool> RPHB_RESET;
    sc_in<bool> RPHB_REG_RST;
    sc_in<bool> IBC_GRANT_EXC;
    sc_in<sc_int<16> > IBC_DATA;
    sc_in<sc_int<4> > RPBI_BPA;
    sc_in<sc_int<2> > RPBI_APA;
    sc_in<bool> MODE_S_P;
    sc_in<sc_int<14> > RHC_ADDR_RT;
    sc_in<bool> RHC_OP_END;
    sc_in<bool> RHC_REG_RST;
    sc_in<sc_int<4> > TM_EXC_FIR;

   // clock port
    sc_in_clk RHC_RPHB_CLK;

    // Output ports
    sc_out<bool> EXC_REQ;
    sc_out<sc_int<14> > EXC_ADDR;
    sc_out<sc_int<16> > EXC_DATA;
    sc_out<bool> EXC_RW;
    sc_out<bool> EXC_IDLE0;
    sc_out<bool> EXC_IDLE1;
    sc_out<sc_int<2> > EXC_RPBH_ID;
    sc_out<sc_int<3> > RHC_OP_CODE;
    sc_out<bool> RHC_OP_START;
    sc_out<sc_int<7> > RHC_SIZE;
    sc_out<sc_int<14> > RHC_ADDR1;
    sc_out<sc_int<14> > RHC_ADDR2;
    sc_out<sc_int<3> > EXC_ERR_NO;

    SC_HAS_PROCESS( EXC_CTRL );

    EXC_CTRL(char *name) : sc_module(name) {
        SC_CTHREAD( start_always, RHC_RPHB_CLK.pos() );
        // reset 
        reset_signal_is( RPHB_RESET, true);
        end_module();
    };
    void start_always();
};
void EXC_CTRL::start_always() { 
    sc_int<16> dt;
    sc_int<16> dtemp;
    sc_int<8> dtemp_1;
    sc_int<14> atemp;
    sc_int<14> atemp_1;
    sc_int<14> atemp_2;
    sc_int<14> atemp_3;
    sc_int<14> atemp_4;
    sc_int<8> timer;
    bool V_reg;
    sc_int<5> DID_reg;
    sc_int<5> MET_reg;
    sc_int<6> Data_Start_reg;
    sc_int<12> total_data_len;
    sc_int<7> link_ptr;
    sc_int<7> index;
    sc_int<2> LEV_reg;
    sc_int<4> DT_reg;
    sc_int<6> NHW_reg;
    sc_int<8> RSN_reg;
    sc_int<8> RP_addr_ptr;
    sc_int<12> cnt;
    sc_int<7> sig_gen_buff_addr;
    sc_int<5> TS0AR;
    sc_int<5> TS1AR;
    bool GB_1_ind;
    bool GB_ext_flg;
    sc_int<2> id_reg;
    sc_int<3> err_no_reg;
    bool rb_flag;
    sc_int<32> _case_1021_;
    sc_int<32> _case_1396_;
    sc_int<16> _case_1934_;
    sc_int<16> _case_2503_;

    sc_int<3> error;
    start: while(1) {

      // reset action
    reset_action: 
         {
	   dt = 0;
	   dtemp_1 = 0;
	   atemp = 0;
	   atemp_1 = 0;
	   atemp_2 = 0;
	   atemp_3 = 0;
	   atemp_4 = 0;
	   timer = 0;
	   V_reg = 0;
	   DID_reg = 0;
	   MET_reg = 0;
	   Data_Start_reg = 0;
	   total_data_len = 0;
	   link_ptr = 0;
	   index = 0;
	   LEV_reg = 0;
	   DT_reg = 0;
	   NHW_reg = 0;
	   RSN_reg = 0;
	   RP_addr_ptr = 0;
	   cnt = 0;
	   sig_gen_buff_addr = 0;
	   TS0AR = 0;
	   TS1AR = 0;
	   GB_1_ind = 0;
	   GB_ext_flg = 0;
	   id_reg = -1;
	   err_no_reg = 0;
	   rb_flag = 0;
	   EXC_IDLE0 = 1;
	   EXC_IDLE1 = 1;
	   error = 0;
	   EXC_REQ = 0;
	   RHC_OP_START = 0;
	 }
    wait();


      // continue with the normal operation

        if(RPHB_REG_RST.read()==1) { // init_reg_op
	   // Initialise programmable register#10, #8, #4, #9, #7, #5 and #6 for hard reset
	   // Reset STATISTIC_0 register(register #10)
	   // `write_mem
            dt = 0;
            atemp = 0;
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) { //wr_01
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            } //wr_01

         // Reset STATISTIC_1 register(register #10)
         // 'write_mem
            atemp = 1;
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) { //wr_02
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            } //wr_02

         // Initialize SSADR_0/POLL_0 register(register #8)
            if(MODE_S_P.read()==1) {
                dt = 0;
            } else {
                dt = 16;
            }

	 // 'write_mem   
            atemp = 2;
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) { // wr_03
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            } // wr_03

	 // Initialize SSADR_1/POLL_1 register(register #8)
	 // 'write_mem
            atemp = 4;
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) { //wr_04
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            } //wr_04
         
	 // Reset RPHB_0 registers(register #4)
            RHC_ADDR1 = 8;
            RHC_ADDR2 = 0;
            RHC_SIZE = 2;
       
	 // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
	 // (end)MACRO: 'set_mem

            EXC_ADDR = 10;
            EXC_DATA = 100;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();

            while(1) { // wr_idle_delay_timer_0
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            }// wr_idle_delay_timer_0

	  // Reset RPHB_1 registers(register #4)
            RHC_ADDR1 = 16;

          // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
         // (end)MACRO: 'set_mem

	 // Initialize NOT_IDLE_TIMER register for RPBH1
	    EXC_ADDR = 18;
            EXC_DATA = 100;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) {// wr_idle_delay_timer_1
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            }// wr_idle_delay_timer_1

	 // Set default value to RHC_PEQ registers(register #9)	    
            RHC_ADDR1 = 20;
            RHC_ADDR2 = 50;
            RHC_SIZE = 4;

         // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
	  // (end)MACRO: 'set_mem

          // Reset RPB_P/RPB_S registers(register #7)
            if(!MODE_S_P.read()==1) {
                atemp = 37;
                atemp_1 = 53;
                RHC_SIZE = 5;
            } else {
                atemp = 35;
                atemp_1 = 51;
                RHC_SIZE = 3;
            }

            RHC_ADDR2 = 0;
            RHC_ADDR1 = 32;

	  // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
	  // (end)MACRO: 'set_mem

            dt = 30;
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) {// wr_05 
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            }// wr_05 

            RHC_ADDR1 = 48;

          // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
         // (end)MACRO: 'set_mem

            atemp = atemp_1;
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) {// wr_06 
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            }// wr_06 

        // Reset RPBH_0/RPBH_1 RP state table(register #5)
            RHC_ADDR1 = 64;
            RHC_ADDR2 = 2;
            RHC_SIZE = 32;

	  // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
	  // (end)MACRO: 'set_mem


            RHC_ADDR1 = 96;

	  // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
	  // (end)MACRO: 'set_mem

          // Reset RPBH_0/RPBH_1 EM list table(register #6)
            RHC_ADDR1 = 128;
            RHC_ADDR2 = 0;

	  // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
	  // (end)MACRO: 'set_mem

            RHC_ADDR1 = 160;

	  // (begin)MACRO: 'set_mem
            RHC_OP_CODE = 2;
            RHC_OP_START = 1;
            wait();
            while(!RHC_OP_END.read()) {
                wait();
                continue;
            }
            wait();
            RHC_OP_START = 0;
            wait();
	  // (end)MACRO: 'set_mem

        } else {
	  wait(); 
        } // init_reg_op

      // Initialise FPP/LPP in all Buffer Queues for both hard and soft reset
        atemp = 270;
        dt = 0;
      
      //buf_empty_loop(begin)
        buf_empty_op: while(1) {
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) {// wr_07
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            }// wr_07

            atemp = (atemp) + 1;
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) {// wr_08
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            }// wr_08
            atemp = (atemp) + 15;
            if((atemp.range(8,4)) >= -4) {
                wait();
                break;
            } else {
                wait();
            }
        }//buf_empty_loop(end)

     // Initialise all Generic Buffers for both hard and soft reset
        atemp = 514;
        V_reg = 0;
        link_ptr = 5;
	dt = (((sc_int<1>(V_reg), sc_int<1>(0)), link_ptr.range(5,0)), sc_int<1>(0));

      // GB_init_loop(begin)
        GB_init_op: while(1) {
            EXC_ADDR = atemp;
            EXC_DATA = dt;
            EXC_RW = 0;
            EXC_REQ = 1;
            wait();
            while(1) { // wr_09 
                EXC_REQ = 0;
                if(IBC_GRANT_EXC.read()==1) {
                    wait();
                    break;
                }
                wait();
            } // wr_09 

            atemp = (atemp) + 128;
            if(((bool)atemp[13]) == 1) {
                wait();
                break;
            } else {
                wait();
            }

            link_ptr = (link_ptr) + 1;
            if(((bool)link_ptr[6]) == 1) {
                link_ptr = 0;
            } else {
                link_ptr = link_ptr;
            }
	    dt = (((sc_int<1>(V_reg), sc_int<1>(0) ), link_ptr.range(5,0)), sc_int<1>(0) );
        } // GB_init_loop(end)

        atemp = -8190;
        dt = 63;

        EXC_ADDR = atemp;
        EXC_DATA = dt;
        EXC_RW = 0;
        EXC_REQ = 1;
        wait();

        while(1) {// wr_09_1
            EXC_REQ = 0;
            if(IBC_GRANT_EXC) {
                wait();
                break;
            }
            wait();
        }// wr_09_1

        wait();

     // RPHB Signals handling part
        main: while(1) {
            wait();
            signal_proc: do{

	      // Set RPBH_0/RPBH_1 ID active
	      // Reset internal error number and flag for return buffer
                id_reg = -1;
                err_no_reg = 0;
                rb_flag = 0;
	      //***************************************************************
	      /* 	#### Modified 19/2/98 EJH #### 
			PEQ code removed from here. PEQ code now only executed if BQ
			is empty
              */
	      //***************************************************************
                wait();
	      // Read the queue(RPHBport -> ExecCtrl)
                RHC_ADDR1 = 256;
	      //MACRO:read_queue(begin)
                RHC_OP_CODE = 0;
                EXC_RPBH_ID = id_reg;
                RHC_OP_START = 1;
                wait();
                while(!RHC_OP_END.read()) {
                    wait();
                    continue;
                }
                wait();
                RHC_OP_START = 0;
                dtemp.range(13,0) = RHC_ADDR_RT;
                wait();
	      //MACRO:read_queue(end)

                sig_gen_buff_addr = dtemp.range(13,7);

                if((sig_gen_buff_addr.range(6,5)) == -1) { // **** If BQ empty
                    wait();

		 // Show idle state for both channels
                    EXC_IDLE0 = 1;
                    EXC_IDLE1 = 1;

                 // Read the queue poll time
                    atemp = 22;
                    EXC_ADDR = atemp;
                    EXC_RPBH_ID = id_reg;
                    EXC_RW = 1;
                    EXC_REQ = 1;
                    wait();
                    while(1) {// rd_01
                        dtemp = IBC_DATA.read();
                        EXC_REQ = 0;
                        if(IBC_GRANT_EXC) {
                            wait();
                            break;
                        }
                        wait();
                    }// rd_01

		  // Set the timer accordingly and wait
                    timer = dtemp.range(7,0);
                    wait();
                    while(timer) {
                        timer = (timer) - 1;
                        wait();
                        continue;
                    }
                    continue;
                } else { // **** BQ contains pointers
                    wait();
                }

	      // Read the header of Generic Buffer addressed by FPP pointer
                atemp = (sig_gen_buff_addr, (sc_int<1>)0);

                EXC_ADDR = atemp;
                EXC_RPBH_ID = id_reg;
                EXC_RW = 1;
                EXC_REQ = 1;
                wait();
                while(1) { // rd_02 
                    dtemp = IBC_DATA.read();
                    EXC_REQ = 0;
                    if(IBC_GRANT_EXC) {
                        wait();
                        break;
                    }
                    wait();
                }// rd_02 

             // Combine with Fault Insertion bit
                DID_reg = (dtemp.range(4,0)) | (((TM_EXC_FIR.read())[1], 0));
                MET_reg = dtemp.range(9,5);
                if((DID_reg) > 2) {
                    err_no_reg = 1;
                    rb_flag = 1;
                    wait();
		 // Show idle state for both channels
                    EXC_IDLE0 = 1;
                    EXC_IDLE1 = 1;
                    break; //goto _signal_proc__END;
                } else {
                    err_no_reg = err_no_reg;
                    rb_flag = rb_flag;
                    wait();
                }

             // Set the no. for RPBH  being handled
                id_reg = (!(DID_reg[0]), !(DID_reg[1]));

	     // Set the idle outputs
                EXC_IDLE0 = DID_reg[1];
                EXC_IDLE1 = DID_reg[0];

             // Read inf0 from the Signal being handled
                atemp = (sig_gen_buff_addr, (sc_int<8>)8);
                EXC_ADDR = atemp;
                EXC_RPBH_ID = id_reg;
                EXC_RW = 1;
                EXC_REQ = 1;
                wait();
                while(1) {// rd_03 
                    dtemp = IBC_DATA.read();
                    EXC_REQ = 0;
                    if(IBC_GRANT_EXC) {
                        wait();
                        break;
                    }
                    wait();
                }// rd_03 

             // Combine with Fault Insertion bit
                if((TM_EXC_FIR.read())[2]) {
                    dtemp = 254;
                } else {
                    dtemp = dtemp;
                }
                wait();

                _case_1021_ = dtemp.range(7,0);
                switch((int)_case_1021_) {
		  // if(193 == _case_1021_) {
                case 193: 
                    if(((MET_reg) != 3) || ((DID_reg) != 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        id_reg = 1;
                        err_no_reg = err_no_reg;
                        rb_flag = rb_flag;
                        //RHC_ADDR1 = (((sc_int<1>(0), sig_gen_buff_addr), RPBI_BPA.read()), sc_int<1>(MODE_S_P.read()));
                        RHC_ADDR1 = (((sc_int<1>(0), sig_gen_buff_addr), sc_int<4>(RPBI_BPA.read())), sc_int<1>(MODE_S_P.read()));
                        RHC_ADDR2 = ((sc_int<8>(10), TS0AR), sc_int<1>(RHC_REG_RST.read()));
                        RHC_OP_CODE = -4;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 272;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<1>(0) );
                        RHC_OP_CODE = 1;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        id_reg = -2;
                        EXC_ADDR = -8192;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        //RHC_ADDR1 = (((sc_int<1>(0), dtemp.range(13,7)), RPBI_BPA.read()), sc_int<1>(MODE_S_P.read()) );
                        RHC_ADDR1 = (((sc_int<1>(0), dtemp.range(13,7)), sc_int<4>(RPBI_BPA.read())), sc_int<1>(MODE_S_P.read()) );
                        RHC_ADDR2 = ((sc_int<8>(11), TS1AR), sc_int<1>(RHC_REG_RST.read()) );
                        RHC_OP_CODE = -4;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 288;
                        RHC_ADDR2 = (dtemp.range(13,7), 0);
                        RHC_OP_CODE = 1;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();}
                        break;
		    // } else if(1 == _case_1021_) {
		case 1:
                    if(((MET_reg) != 3) || ((DID_reg) != 1)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        err_no_reg = err_no_reg;
                        rb_flag = 1;
                        atemp = (sig_gen_buff_addr, sc_int<8>(10));
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        TS0AR = dtemp.range(4,0);
                        TS1AR = (TS0AR) + 1;
                        atemp = (sig_gen_buff_addr, sc_int<8>(11));
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        dt = dtemp;
                        atemp = 9;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        dt = (dt) + 32;
                        atemp = 17;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                    }
		    break;
		    // } else if((((194 == _case_1021_) || (201 == _case_1021_)) || (205 == _case_1021_)) || (204 == _case_1021_)) {
		  case  194:;
		  case  201:;
		  case  205:;
		  
                    if(((MET_reg) != 3) || ((DID_reg) != 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        err_no_reg = err_no_reg;
                        rb_flag = 1;
                        wait();
                    }
		  break;
		  // } else if(209 == _case_1021_) {
		  case 209:
                    if(((MET_reg) != 3) || ((DID_reg) == 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        err_no_reg = err_no_reg;
                        rb_flag = 1;
                        wait();
                    }
		  break;
		 //} else if(7 == _case_1021_) {
	 	 case 7:
                    if(((MET_reg) != 3) || ((DID_reg) == 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        err_no_reg = err_no_reg;
                        rb_flag = rb_flag;
                        if((DID_reg[0]) == 1) {
                            DID_reg = 10;
                            atemp_1 = 272;
                        } else {
                            DID_reg = 11;
                            atemp_1 = 288;
                        }
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<1>(0));
                        RHC_ADDR2 = 0;
                        RHC_SIZE = ((sc_int<1>(0), DID_reg), sc_int<1>(0));
                        RHC_OP_CODE = -3;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 451;
                        RHC_ADDR2 = 0;
                        RHC_SIZE = sig_gen_buff_addr;
                        RHC_OP_CODE = -2;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        atemp = (sig_gen_buff_addr, sc_int<8>(11) );
                        dt = 1;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        atemp = (sig_gen_buff_addr, sc_int<8>(12) );
                        dt = 0;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        RHC_ADDR1 = atemp_1;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<1>(0) );
                        RHC_OP_CODE = 1;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                    }
		    break;
		  //  } else if(59 == _case_1021_) {
		 case 59:
                    if(((MET_reg) != 3) || ((DID_reg) == 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        err_no_reg = err_no_reg;
                        rb_flag = rb_flag;
                        atemp = (sig_gen_buff_addr, sc_int<8>(10) );
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        RP_addr_ptr = dtemp.range(7,0);
                        if((DID_reg[0]) == 1) {
                            RP_addr_ptr = (sc_int<8>(2), RP_addr_ptr.range(4,0));
                            DID_reg = 10;
                            atemp_1 = (sc_int<8>(4), RP_addr_ptr.range(4,0));
                            atemp_2 = 9;
                            atemp_3 = 272;
                        } else {
                            RP_addr_ptr = (sc_int<8>(3), RP_addr_ptr.range(4,0));
                            DID_reg = 11;
                            atemp_1 = (sc_int<8>(5), RP_addr_ptr.range(4,0));
                            atemp_2 = 17;
                            atemp_3 = 288;
                        }
                        atemp = (sig_gen_buff_addr, sc_int<8>(11) );
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
		      }
                        _case_1396_ = int(dtemp);
			switch((int)_case_1396_) {
		     // if(1 == _case_1396_) {
			case 1:
                            rb_flag = 1;
                            atemp = (sc_int<1>(0), RP_addr_ptr);
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            dt = dtemp | 2;
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            } 
			    break;
			    // } else if(2 == _case_1396_) {
			case 2:
                            rb_flag = 1;
                            atemp = (sc_int<1>(0), RP_addr_ptr);
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            dt = dtemp & -3;
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
			    break;
			  // } else if(3 == _case_1396_) {
			case 3:
                            rb_flag = 1;
                            atemp = (sig_gen_buff_addr, sc_int<8>(12) );
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            atemp = atemp_1;
                            dt = dtemp;
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            } break;
			    // } else if(4 == _case_1396_) {
			case 4:
                            atemp = atemp_2;
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            RHC_ADDR1 = (sig_gen_buff_addr, sc_int<1>(0) );
                            RHC_ADDR2 = 0;
                            RHC_SIZE = ((sc_int<1>(0), DID_reg), sc_int<1>(0) );
                            RHC_OP_CODE = -3;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait();
                            RHC_ADDR1 = 3779;
                            RHC_ADDR2 = 0;
                            RHC_SIZE = sig_gen_buff_addr;
                            RHC_OP_CODE = -2;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait();
                            atemp = (sig_gen_buff_addr, (sc_int<8>)10);
                            dt = (dtemp.range(15,5), RP_addr_ptr.range(4,0));
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            atemp = (sc_int<1>(0), RP_addr_ptr);
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            atemp = (sig_gen_buff_addr, sc_int<8>(11) );
                            if(!(dtemp[1])) {
                                dt = 0;
                            } else {
                                dt = 1;
                            }
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            atemp = atemp_1;
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            atemp = (sig_gen_buff_addr, sc_int<8>(12));
                            dt = dtemp;
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            RHC_ADDR1 = atemp_3;
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<1>(0) );
                            RHC_OP_CODE = 1;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait();
			    break;
			    // } else {
			default:
                            err_no_reg = -4;
                            rb_flag = 1;
                            wait();
                    } // end _case_1396_
                    
		  // } else if(208 == _case_1021_) {
		   case 208:
                    if(((MET_reg) != 3) || ((DID_reg) == 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        err_no_reg = err_no_reg;
                        rb_flag = rb_flag;
                        atemp = (sig_gen_buff_addr, sc_int<8>(24) );
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        NHW_reg = dtemp.range(13,8);
                        Data_Start_reg = (NHW_reg) + 2;
                        atemp = (sig_gen_buff_addr, sc_int<8>(8));
                        dt = dtemp;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        atemp = (sig_gen_buff_addr, sc_int<8>(25) );
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        total_data_len = dtemp.range(11,0);
                        atemp = (sig_gen_buff_addr, sc_int<8>(9) );
                        dt = dtemp;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        Data_Start_reg = 32 + ((Data_Start_reg) << 1);
                        MET_reg = 6;
                        if(DID_reg[0]) {
                            DID_reg = 10;
                            atemp_1 = 272;
                        } else {
                            DID_reg = 11;
                            atemp_1 = 288;
                        }
                        atemp = (sig_gen_buff_addr, sc_int<1>(0) );
                        dt = ((Data_Start_reg, MET_reg), DID_reg);
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        atemp = (sig_gen_buff_addr, sc_int<1>(1));
                        dt = (sc_int<1>(0), total_data_len);
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(26));
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(10));
                        RHC_SIZE = NHW_reg;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = atemp_1;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<1>(0) );
                        RHC_OP_CODE = 1;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                    }
		    break;
		   // } else if(9 == _case_1021_) {
		   case 9:
                    if(((MET_reg) != 3) || ((DID_reg) == 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        err_no_reg = err_no_reg;
                        rb_flag = rb_flag;
                        Data_Start_reg = -32;
                        MET_reg = 5;
                        total_data_len = 2;
                        LEV_reg = 0;
                        NHW_reg = 3;
                        RSN_reg = 9;
                        DT_reg = 0;
                        link_ptr = 0;
                        V_reg = 0;
                        if(DID_reg[0]) {
                            DID_reg = 10;
                            atemp_1 = 272;
                        } else {
                            DID_reg = 11;
                            atemp_1 = 288;
                        }
                        atemp = (sig_gen_buff_addr, sc_int<8>(10) );
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        if((TM_EXC_FIR.read())[3]) {
                            dtemp = 4096;
                        } else {
                            dtemp = dtemp;
                        }
                        wait();
                        _case_1934_ = dtemp;
			switch((int)_case_1934_) {
			  case 0:
			  // if(0 == _case_1934_) {
                            atemp = (sig_gen_buff_addr, sc_int<8>(24) );
			    dt = (sc_int<10>(256), MODE_S_P);
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            } break;
			    // } else if(1 == _case_1934_) {
			case 1:
                            atemp = (sig_gen_buff_addr, sc_int<8>(24) );
                            dt = 3;
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            } break;
			    // } else if(2 == _case_1934_) {
			case 2:
                            atemp = (sig_gen_buff_addr, sc_int<8>(24) );
                            dt = 32;
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            } break;
			  // } else if(3 == _case_1934_) {
			case 3: 
                            atemp = (sig_gen_buff_addr, sc_int<8>(24) );
                            dt = ( sc_int<1>(0), ( RPBI_BPA.read(), RPBI_APA.read() ) );
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            } break;
			    // } else if(4 == _case_1934_) {
			case 4:
                            total_data_len = 6;
                            if(!(DID_reg[0])) {
                                RHC_ADDR1 = 8;
                            } else {
                                RHC_ADDR1 = 16;
                            }
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24) );
                            RHC_SIZE = 3;
                            RHC_OP_CODE = 3;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait(); break;
			    //  } else if(5 == _case_1934_) {
			case 5:
                            total_data_len = 64;
                            if(!(DID_reg[0])) {
                                RHC_ADDR1 = 64;
                            } else {
                                RHC_ADDR1 = 96;
                            }
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24) );
                            RHC_SIZE = 32;
                            RHC_OP_CODE = 3;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait(); break;
			    // } else if(6 == _case_1934_) {
			case 6:
                            total_data_len = 64;
                            if(!(DID_reg[0])) {
                                RHC_ADDR1 = 128;
                            } else {
                                RHC_ADDR1 = 160;
                            }
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24));
                            RHC_SIZE = 32;
                            RHC_OP_CODE = 3;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait(); break;
			    //  } else if(7 == _case_1934_) {
			case 7:
                            if(!MODE_S_P.read()) {
                                total_data_len = 12;
                                RHC_SIZE = 6;
                            } else {
                                total_data_len = 8;
                                RHC_SIZE = 4;
                            }
                            if(!(DID_reg[0])) {
                                RHC_ADDR1 = 32;
                            } else {
                                RHC_ADDR1 = 48;
                            }
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24));
                            RHC_OP_CODE = 3;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait(); break;
			    // } else if(8 == _case_1934_) {
			case 8:
                            total_data_len = 2;
                            if(!(DID_reg[0])) {
                                RHC_ADDR1 = 2;
                            } else {
                                RHC_ADDR1 = 4;
                            }
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24));
                            RHC_SIZE = 1;
                            RHC_OP_CODE = 3;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait(); break;
			    // } else if(9 == _case_1934_) {
			case 9:
                            total_data_len = 8;
                            RHC_ADDR1 = 20;
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24) );
                            RHC_SIZE = 4;
                            RHC_OP_CODE = 3;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait(); break;
			    //  } else if(10 == _case_1934_) {
			case 10:
                            total_data_len = 2;
                            if(!(DID_reg[0])) {
                                atemp_2 = 0;
                            } else {
                                atemp_2 = 1;
                            }
                            RHC_ADDR1 = atemp_2;
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24) );
                            RHC_SIZE = 1;
                            RHC_OP_CODE = 3;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait();
                            atemp = atemp_2;
                            dt = 0;
                            EXC_ADDR = atemp;
                            EXC_DATA = dt;
                            EXC_RW = 0;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            } break;
			    // } else if(-1 == _case_1934_) {
			case -1:
                            index = 0;
                            GB_1_ind = 1;
                            atemp = (sig_gen_buff_addr, sc_int<8>(11) );
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            atemp_2 = dtemp.range(13,0);
                            atemp_3 = (sig_gen_buff_addr, sc_int<8>(24) );
                            atemp = (sig_gen_buff_addr, sc_int<8>(13) );
                            EXC_ADDR = atemp;
                            EXC_RPBH_ID = id_reg;
                            EXC_RW = 1;
                            EXC_REQ = 1;
                            wait();
                            while(1) {
                                dtemp = IBC_DATA.read();
                                EXC_REQ = 0;
                                if(IBC_GRANT_EXC) {
                                    wait();
                                    break;
                                }
                                wait();
                            }
                            cnt = dtemp.range(11,0);
                            total_data_len = dtemp.range(11,0);
                            dtemp_1 = -48;
                            mem_copy_op_1: while(1) {
                                if((cnt) <= 1) {
                                    index = 0;
                                    wait();
                                    break;
                                } else {
                                    wait();
                                }
                                if(cnt > (dtemp_1)) {
                                    index = dtemp_1.range(7,1);
                                    GB_ext_flg = 1;
                                    cnt = cnt - (dtemp_1);
                                } else {
                                    index = cnt.range(7,1);
                                    GB_ext_flg = 0;
                                    cnt = cnt & 1;
                                }
                                RHC_ADDR1 = atemp_2;
                                RHC_ADDR2 = atemp_3;
                                RHC_SIZE = index;
                                RHC_OP_CODE = 3;
                                EXC_RPBH_ID = id_reg;
                                RHC_OP_START = 1;
                                wait();
                                while(!RHC_OP_END.read()) {
                                    wait();
                                    continue;
                                }
                                wait();
                                RHC_OP_START = 0;
                                wait();
                                if(!GB_ext_flg) {
                                    wait();
                                    break;
                                } else {
                                    wait();
                                }
                                EXC_ADDR = -8192;
                                EXC_RPBH_ID = id_reg;
                                EXC_RW = 1;
                                EXC_REQ = 1;
                                wait();
                                while(1) {
                                    dtemp = IBC_DATA.read();
                                    EXC_REQ = 0;
                                    if(IBC_GRANT_EXC) {
                                        wait();
                                        break;
                                    }
                                    wait();
                                }
                                if(GB_1_ind) {
                                    link_ptr = dtemp.range(13,7);
                                    V_reg = 1;
                                    atemp = atemp;
                                    dt = dt;
                                    wait();
                                } else {
                                    link_ptr = link_ptr;
                                    V_reg = V_reg;
                                    atemp = (atemp_4.range(13,7), sc_int<8>(2));
                                    dt = (sc_int<8>(-4), dtemp.range(12,0));
                                    EXC_ADDR = atemp;
                                    EXC_DATA = dt;
                                    EXC_RW = 0;
                                    EXC_REQ = 1;
                                    wait();
                                    while(1) {
                                        EXC_REQ = 0;
                                        if(IBC_GRANT_EXC) {
                                            wait();
                                            break;
                                        }
                                        wait();
                                    }
                                }
                                dtemp_1 = -16;
                                atemp_2 = atemp_2 + (index);
                                atemp_3 = (dtemp.range(13,7), sc_int<7>( 8 ));
                                atemp_4 = dtemp.range(13,0);
                                GB_1_ind = 0;
                            }
                            if(!(cnt[0])) {
                                wait();
                            } else {
                                wait();
                                atemp = atemp_2 + (index);
                                EXC_ADDR = atemp;
                                EXC_RPBH_ID = id_reg;
                                EXC_RW = 1;
                                EXC_REQ = 1;
                                wait();
                                while(1) {
                                    dtemp = IBC_DATA.read();
                                    EXC_REQ = 0;
                                    if(IBC_GRANT_EXC) {
                                        wait();
                                        break;
                                    }
                                    wait();
                                }
                                atemp = atemp_3 + (index);
                                dt = (sc_int<1>(0), dtemp.range(7,0));
                                EXC_ADDR = atemp;
                                EXC_DATA = dt;
                                EXC_RW = 0;
                                EXC_REQ = 1;
                                wait();
                                while(1) {
                                    EXC_REQ = 0;
                                    if(IBC_GRANT_EXC) {
                                        wait();
                                        break;
                                    }
                                    wait();
                                }
                            }
                            if(!GB_1_ind) {
                                atemp = (atemp_4.range(13,7), sc_int<7>( 2 ));
                                dt = 0;
                                EXC_ADDR = atemp;
                                EXC_DATA = dt;
                                EXC_RW = 0;
                                EXC_REQ = 1;
                                wait();
                                while(1) {
                                    EXC_REQ = 0;
                                    if(IBC_GRANT_EXC) {
                                        wait();
                                        break;
                                    }
                                    wait();
                                }
                            } else {
                                wait();
                            } break;
			    // } else {
			default:
                            err_no_reg = -3;
                            rb_flag = 1;
                            wait();
                        } // end _case_1934_
                        if((rb_flag) == 0) {
                            wait();
                            RHC_ADDR1 = (sig_gen_buff_addr, link_ptr);
                            RHC_ADDR2 = (sc_int<1>(0), total_data_len);
                            RHC_SIZE = ((sc_int<1>(0), DID_reg), V_reg);
                            RHC_OP_CODE = -3;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait();
                            RHC_ADDR1 = (sc_int<8>(9), NHW_reg);
                            RHC_ADDR2 = (sc_int<1>(0), total_data_len);
                            RHC_SIZE = sig_gen_buff_addr;
                            RHC_OP_CODE = -2;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait();
                            RHC_ADDR1 = atemp_1;
                            RHC_ADDR2 = (sig_gen_buff_addr, sc_int<1>(0) );
                            RHC_OP_CODE = 1;
                            EXC_RPBH_ID = id_reg;
                            RHC_OP_START = 1;
                            wait();
                            while(!RHC_OP_END.read()) {
                                wait();
                                continue;
                            }
                            wait();
                            RHC_OP_START = 0;
                            wait();
                        } else {
                            wait();
                        }
                    }
		    break;
		  // } else if(210 == _case_1021_) {
		case 210: 
                    if(((MET_reg) != 3) || ((DID_reg) == 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        err_no_reg = err_no_reg;
                        rb_flag = 1;
                        atemp = (sig_gen_buff_addr, sc_int<1>(1));
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        cnt = dtemp.range(11,0);
                        atemp = (sig_gen_buff_addr, sc_int<8>(10) );
                        EXC_ADDR = atemp;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        if((TM_EXC_FIR.read())[3]) {
                            dtemp = 4096;
                        } else {
                            dtemp = dtemp;
                        }
                        wait();
                        _case_2503_ = dtemp;
			switch((int)_case_2503_) {
			  // if(4 == _case_2503_) {
			case 4:
                            if((cnt) != 6) {
                                err_no_reg = 2;
                                wait();
                            } else {
                                atemp = (sig_gen_buff_addr, sc_int<8>(24) );
                                EXC_ADDR = atemp;
                                EXC_RPBH_ID = id_reg;
                                EXC_RW = 1;
                                EXC_REQ = 1;
                                wait();
                                while(1) {
                                    dtemp = IBC_DATA.read();
                                    EXC_REQ = 0;
                                    if(IBC_GRANT_EXC) {
                                        wait();
                                        break;
                                    }
                                    wait();
                                }
                                dt = (sc_int<1>(1), dtemp.range(14,0));
                                if(DID_reg[0]) {
                                    atemp = 8;
                                    atemp_1 = 9;
                                } else {
                                    atemp = 16;
                                    atemp_1 = 17;
                                }
                                EXC_ADDR = atemp;
                                EXC_DATA = dt;
                                EXC_RW = 0;
                                EXC_REQ = 1;
                                wait();
                                while(1) {
                                    EXC_REQ = 0;
                                    if(IBC_GRANT_EXC) {
                                        wait();
                                        break;
                                    }
                                    wait();
                                }
                                RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(25) );
                                RHC_ADDR2 = atemp_1;
                                RHC_SIZE = 2;
                                RHC_OP_CODE = 3;
                                EXC_RPBH_ID = id_reg;
                                RHC_OP_START = 1;
                                wait();
                                while(!RHC_OP_END.read()) {
                                    wait();
                                    continue;
                                }
                                wait();
                                RHC_OP_START = 0;
                                wait();
                            } break;
			    // } else if(5 == _case_2503_) {
			case 5:
                            if((cnt) != 64) {
                                err_no_reg = 2;
                                wait();
                            } else {
                                RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(24) );
                                RHC_SIZE = 32;
                                if(DID_reg[0]) {
                                    RHC_ADDR2 = 64;
                                } else {
                                    RHC_ADDR2 = 96;
                                }
                                RHC_OP_CODE = 3;
                                EXC_RPBH_ID = id_reg;
                                RHC_OP_START = 1;
                                wait();
                                while(!RHC_OP_END.read()) {
                                    wait();
                                    continue;
                                }
                                wait();
                                RHC_OP_START = 0;
                                wait();
                            } break;
			    // } else if(6 == _case_2503_) {
			case 6:
                            if((cnt) != 64) {
                                err_no_reg = 2;
                                wait();
                            } else {
                                RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(24) );
                                RHC_SIZE = 32;
                                if(DID_reg[0]) {
                                    RHC_ADDR2 = 128;
                                } else {
                                    RHC_ADDR2 = 160;
                                }
                                RHC_OP_CODE = 3;
                                EXC_RPBH_ID = id_reg;
                                RHC_OP_START = 1;
                                wait();
                                while(!RHC_OP_END.read()) {
                                    wait();
                                    continue;
                                }
                                wait();
                                RHC_OP_START = 0;
                                wait();
                            } break;
			    //  } else if(7 == _case_2503_) {
			case 7:
                            if(((!MODE_S_P.read()) && ((cnt) != 12)) || (MODE_S_P.read() && ((cnt) != 8))) {
                                err_no_reg = 2;
                                wait();
                            } else {
                                RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(24) );
                                if(!MODE_S_P.read()) {
                                    RHC_SIZE = 6;
                                } else {
                                    RHC_SIZE = 4;
                                }
                                if(DID_reg[0]) {
                                    RHC_ADDR2 = 32;
                                } else {
                                    RHC_ADDR2 = 48;
                                }
                                RHC_OP_CODE = 3;
                                EXC_RPBH_ID = id_reg;
                                RHC_OP_START = 1;
                                wait();
                                while(!RHC_OP_END.read()) {
                                    wait();
                                    continue;
                                }
                                wait();
                                RHC_OP_START = 0;
                                wait();
                            } break;
			    // } else if(8 == _case_2503_) {
			case 8:
                            if((cnt) != 2) {
                                err_no_reg = 2;
                                wait();
                            } else {
                                atemp = (sig_gen_buff_addr, sc_int<8>(24) );
                                if(DID_reg[0]) {
                                    atemp_1 = 2;
                                } else {
                                    atemp_1 = 4;
                                }
                                RHC_ADDR1 = atemp;
                                RHC_ADDR2 = atemp_1;
                                RHC_SIZE = 1;
                                RHC_OP_CODE = 3;
                                EXC_RPBH_ID = id_reg;
                                RHC_OP_START = 1;
                                wait();
                                while(!RHC_OP_END.read()) {
                                    wait();
                                    continue;
                                }
                                wait();
                                RHC_OP_START = 0;
                                wait();
                            } break;
			    // } else if(9 == _case_2503_) {
			case 9:
                            if((cnt) != 8) {
                                err_no_reg = 2;
                                wait();
                            } else {
                                RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(24) );
                                RHC_ADDR2 = 20;
                                RHC_SIZE = 4;
                                RHC_OP_CODE = 3;
                                EXC_RPBH_ID = id_reg;
                                RHC_OP_START = 1;
                                wait();
                                while(!RHC_OP_END.read()) {
                                    wait();
                                    continue;
                                }
                                wait();
                                RHC_OP_START = 0;
                                wait();
                            } break;
			    // } else if(-1 == _case_2503_) {
			case -1:
                            wt_mem: do {
                                atemp = (sig_gen_buff_addr, sc_int<8>(11) );
                                EXC_ADDR = atemp;
                                EXC_RPBH_ID = id_reg;
                                EXC_RW = 1;
                                EXC_REQ = 1;
                                wait();
                                while(1) {
                                    dtemp = IBC_DATA.read();
                                    EXC_REQ = 0;
                                    if(IBC_GRANT_EXC) {
                                        wait();
                                        break;
                                    }
                                    wait();
                                }
                                atemp_2 = dtemp.range(13,0);
                                atemp_1 = (sig_gen_buff_addr, sc_int<8>(24) );
                                GB_copy_op_1: while(1) {
                                    if((cnt) <= 1) {
                                        wait();
                                        break;
                                    } else {
                                        atemp = atemp_1;
                                        EXC_ADDR = atemp;
                                        EXC_RPBH_ID = id_reg;
                                        EXC_RW = 1;
                                        EXC_REQ = 1;
                                        wait();
                                        while(1) {
                                            dtemp = IBC_DATA.read();
                                            EXC_REQ = 0;
                                            if(IBC_GRANT_EXC) {
                                                wait();
                                                break;
                                            }
                                            wait();
                                        }
                                        atemp = atemp_2;
                                        dt = dtemp;
                                        EXC_ADDR = atemp;
                                        EXC_DATA = dt;
                                        EXC_RW = 0;
                                        EXC_REQ = 1;
                                        wait();
                                        while(1) {
                                            EXC_REQ = 0;
                                            if(IBC_GRANT_EXC) {
                                                wait();
                                                break;
                                            }
                                            wait();
                                        }
                                        atemp_1 = (atemp_1) + 1;
                                        atemp_2 = (atemp_2) + 1;
                                        cnt = (cnt) - 2;
                                        if(atemp_1 <= ((sig_gen_buff_addr, sc_int<1>(-1) ))) {
                                            wait();
                                            continue;
                                        } else {
                                            atemp = (sig_gen_buff_addr, sc_int<7>( 2 ));
                                            EXC_ADDR = atemp;
                                            EXC_RPBH_ID = id_reg;
                                            EXC_RW = 1;
                                            EXC_REQ = 1;
                                            wait();
                                            while(1) {
                                                dtemp = IBC_DATA.read();
                                                EXC_REQ = 0;
                                                if(IBC_GRANT_EXC) {
                                                    wait();
                                                    break;
                                                }
                                                wait();
                                            }
                                            dt = ((sc_int<1>(0), sig_gen_buff_addr), sc_int<1>(0) );
                                            EXC_ADDR = -8192;
                                            EXC_DATA = dt;
                                            EXC_RW = 0;
                                            EXC_REQ = 1;
                                            wait();
                                            while(1) {
                                                EXC_REQ = 0;
                                                if(IBC_GRANT_EXC) {
                                                    wait();
                                                    break;
                                                }
                                                wait();
                                            }
                                            sig_gen_buff_addr = dtemp.range(13,7);
                                            atemp_1 = (sig_gen_buff_addr, sc_int<8>(8) );
                                        }
                                    }
                                }
                                if(!(cnt[0])) {
                                    wait();
                                    break; //goto _wt_mem__END;
                                } else {
                                    atemp = atemp_1;
                                    EXC_ADDR = atemp;
                                    EXC_RPBH_ID = id_reg;
                                    EXC_RW = 1;
                                    EXC_REQ = 1;
                                    wait();
                                    while(1) {
                                        dtemp = IBC_DATA.read();
                                        EXC_REQ = 0;
                                        if(IBC_GRANT_EXC) {
                                            wait();
                                            break;
                                        }
                                        wait();
                                    }
                                    atemp_3.range(7,0) = dtemp.range(7,0);
                                    atemp = atemp_2;
                                    EXC_ADDR = atemp;
                                    EXC_RPBH_ID = id_reg;
                                    EXC_RW = 1;
                                    EXC_REQ = 1;
                                    wait();
                                    while(1) {
                                        dtemp = IBC_DATA.read();
                                        EXC_REQ = 0;
                                        if(IBC_GRANT_EXC) {
                                            wait();
                                            break;
                                        }
                                        wait();
                                    }
                                    dt = (dtemp.range(15,8), atemp_3.range(7,0));
                                    EXC_ADDR = atemp;
                                    EXC_DATA = dt;
                                    EXC_RW = 0;
                                    EXC_REQ = 1;
                                    wait();
                                    while(1) {
                                        EXC_REQ = 0;
                                        if(IBC_GRANT_EXC) {
                                            wait();
                                            break;
                                        }
                                        wait();
                                    }
                                }
                            _wt_mem__END:; }while(0);
			    break;
			//  } else {
			default:
                            err_no_reg = -3;
                            wait();
                        } // end _case_2503_
                    }
		    break;
		    //  } else if(176 == _case_1021_) {
		case 176:
                    if(((MET_reg) != 3) || ((DID_reg) != 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        wait();
                        rb_flag = rb_flag;
                        id_reg = 1;
                        if(!MODE_S_P.read()) {
                            total_data_len = 156;
                        } else {
                            total_data_len = 152;
                        }
                        RSN_reg = 10;
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<1>(0) );
                        RHC_ADDR2 = (sc_int<1>(0) , total_data_len);
                        RHC_SIZE = 20;
                        RHC_OP_CODE = -3;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (RSN_reg, sc_int<1>(1) );
                        RHC_ADDR2 = (sc_int<1>(0), total_data_len);
                        RHC_SIZE = sig_gen_buff_addr;
                        RHC_OP_CODE = -2;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        atemp = (sig_gen_buff_addr, sc_int<8>(10) );
                        dt = 0;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        RHC_ADDR1 = 8;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24) );
                        RHC_SIZE = 3;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 64;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(27) );
                        RHC_SIZE = 32;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 128;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(59) );
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 32;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(-37) );
                        if(!MODE_S_P.read()) {
                            atemp_1 = (sig_gen_buff_addr, sc_int<8>(-31) );
                            atemp_2 = (sig_gen_buff_addr, sc_int<8>(-30) );
                            atemp_3 = 6;
                            RHC_SIZE = 6;
                        } else {
                            atemp_1 = (sig_gen_buff_addr, sc_int<8>(-33) );
                            atemp_2 = (sig_gen_buff_addr, sc_int<8>(-32) );
                            atemp_3 = 4;
                            RHC_SIZE = 4;
                        }
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 2;
                        RHC_ADDR2 = atemp_1;
                        RHC_SIZE = 1;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 20;
                        RHC_ADDR2 = atemp_2;
                        RHC_SIZE = 4;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 272;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<1>(0) );
                        RHC_OP_CODE = 1;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        id_reg = -2;
                        EXC_ADDR = -8192;
                        EXC_RPBH_ID = id_reg;
                        EXC_RW = 1;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            dtemp = IBC_DATA.read();
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        sig_gen_buff_addr = dtemp.range(13,7);
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<1>(0) );
                        RHC_ADDR2 = (sc_int<1>(0), total_data_len);
                        RHC_SIZE = 22;
                        RHC_OP_CODE = -3;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (RSN_reg, sc_int<1>(1) );
                        RHC_ADDR2 = (sc_int<1>(0) , total_data_len);
                        RHC_SIZE = sig_gen_buff_addr;
                        RHC_OP_CODE = -2;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        atemp = (sig_gen_buff_addr, sc_int<8>(10) );
                        dt = 0;
                        EXC_ADDR = atemp;
                        EXC_DATA = dt;
                        EXC_RW = 0;
                        EXC_REQ = 1;
                        wait();
                        while(1) {
                            EXC_REQ = 0;
                            if(IBC_GRANT_EXC) {
                                wait();
                                break;
                            }
                            wait();
                        }
                        RHC_ADDR1 = 16;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(24) );
                        RHC_SIZE = 3;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 96;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(27) );
                        RHC_SIZE = 32;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 160;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(59) );
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 48;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<8>(-37));
                        RHC_SIZE = atemp_3.range(6,0);
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 4;
                        RHC_ADDR2 = (sig_gen_buff_addr.range(6,0), atemp_1.range(6,0));
                        RHC_SIZE = 1;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 20;
                        RHC_ADDR2 = (sig_gen_buff_addr.range(6,0), atemp_2.range(6,0));
                        RHC_SIZE = 4;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 288;
                        RHC_ADDR2 = (sig_gen_buff_addr.range(6,0), 0);
                        RHC_OP_CODE = 1;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                    }
		  break;
		  // } else if(10 == _case_1021_) {
		case 10:
                    if(((MET_reg) != 3) || ((DID_reg) == 0)) {
                        err_no_reg = 1;
                        rb_flag = 1;
                        wait();
                    } else {
                        err_no_reg = err_no_reg;
                        rb_flag = rb_flag;
                        wait();
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(24) );
                        if(DID_reg[0]) {
                            atemp_1 = 128;
                            atemp_2 = 32;
                            atemp_3 = 2;
                            atemp_4 = 272;
                            atemp = 64;
                            DID_reg = 10;
                            RHC_ADDR2 = 8;
                        } else {
                            atemp_1 = 160;
                            atemp_2 = 48;
                            atemp_3 = 4;
                            atemp_4 = 288;
                            atemp = 96;
                            DID_reg = 11;
                            RHC_ADDR2 = 16;
                        }
                        RHC_SIZE = 3;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(27) );
                        RHC_ADDR2 = atemp;
                        RHC_SIZE = 32;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(59));
                        RHC_ADDR2 = atemp_1;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<8>(-37) );
                        RHC_ADDR2 = atemp_2;
                        if(!MODE_S_P.read()) {
                            atemp_1 = -3743;
                            RHC_SIZE = 6;
                        } else {
                            atemp_1 = -4001;
                            RHC_SIZE = 4;
                        }
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (sig_gen_buff_addr.range(6,0), atemp_1.range(6,0));
                        RHC_ADDR2 = atemp_3;
                        RHC_SIZE = 1;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (sig_gen_buff_addr.range(6,0), atemp_1.range(13,7));
                        RHC_ADDR2 = 20;
                        RHC_SIZE = 4;
                        RHC_OP_CODE = 3;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = (sig_gen_buff_addr, sc_int<1>(0) );
                        RHC_ADDR2 = 0;
                        RHC_SIZE = ((sc_int<1>(0), DID_reg), sc_int<1>(0) );
                        RHC_OP_CODE = -3;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();
                        RHC_ADDR1 = 768;
                        RHC_ADDR2 = 0;
                        RHC_SIZE = sig_gen_buff_addr;
                        RHC_OP_CODE = -2;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
                        RHC_OP_START = 0;
                        wait();

		     //Write Buffer Queue	
                        RHC_ADDR1 = atemp_4;
                        RHC_ADDR2 = (sig_gen_buff_addr, sc_int<1>(0) );

		     //Macro:write_queue(begin)
                        RHC_OP_CODE = 1;
                        EXC_RPBH_ID = id_reg;
                        RHC_OP_START = 1;
                        wait();
                        while(!RHC_OP_END.read()) {
                            wait();
                            continue;
                        }
                        wait();
		     //Macro:write_queue(end)
                        RHC_OP_START = 0;
                        wait();
		         } 
		  break;
	       // } else {
		default: 
                    err_no_reg = 3;
                    rb_flag = 1;
                    wait();
                
	    } // end  _case_1021_

            _signal_proc__END:; } while(0);
            if((err_no_reg) != 0) {
                error = err_no_reg;
                wait();
                error = 0;
                wait();
            } else {
                wait();
            }
            if((rb_flag) != 0) {
                dt = ((sc_int<1>(0), sig_gen_buff_addr), sc_int<1>(0));
                EXC_ADDR = -8192;
                EXC_DATA = dt;
                EXC_RW = 0;
                EXC_REQ = 1;
                wait();
                while(1) {
                    EXC_REQ = 0;
                    if(IBC_GRANT_EXC) {
                        wait();
                        break;
                    }
                    wait();
                }
            } else {
                wait();
            }
		} // end start
	    } // end main
	} // main entry
       
int
sc_main( int argc, char* argv[] )
{
    return 0;
}
