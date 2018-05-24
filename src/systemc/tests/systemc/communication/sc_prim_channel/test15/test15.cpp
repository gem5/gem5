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

  test15.cpp -- 

  Original Author: Ucar Aziz, Synopsys, Inc., 2002-02-15
                   Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

// test of sc_prim_channel::wait(double, sc_time_unit, sc_event_and_list&)

#include <systemc.h>

//write and read interfaces
class write_if : virtual public
sc_interface
 {
  public:
   virtual void write() = 0;
};

class read_if : virtual public 
sc_interface
{
 public:
  virtual void read( ) = 0;
};

// channel implements write_if and read_if interfaces
class channel :
  public sc_channel,
  public write_if,
  public read_if
{

  public :

  //constructor
  channel(sc_module_name name):sc_channel(name)  , data(0)
  { }

  //write to channel 
  void write(){
    int i = 0;

    while(1){
    wait(10, SC_NS);
    data = i;
    cout <<"simulation time" << ":" << sc_time_stamp()<<"    ";
    cout<<"writing "<< data <<" to channel" << endl;
  
    if(i < 3){
      write_event_1.notify(20, SC_NS);
    }
    else if(3 <= i && i < 6) {
      write_event_2.notify(5, SC_NS);
	}
    else{
      write_event_2.notify(5, SC_NS);
      write_event_1.notify(5, SC_NS);
    }
    i++;
    }
  }
  //read from channel
  void read( ){
    int j;

    while(1){
    wait(10, SC_NS, write_event_1 & write_event_2);
    j = data;
    cout <<"simulation time" << ":" << sc_time_stamp();
    cout<<"    reading "<<j<<" from channel" << endl;
   }
  }

  private:
  int data;
  sc_event write_event_1, write_event_2;

};

//source module
SC_MODULE(mod_a)
{
  sc_port<write_if> out;

  void write( )
  {
    out->write();
  }  

  SC_CTOR( mod_a ){
  
    SC_THREAD(write);
  }
};
  
//sink module
SC_MODULE(mod_b)
{
  sc_port<read_if> input;
  int i;

  void read( )
  {
   input->read();
  }  

  SC_CTOR( mod_b ){
  
    SC_THREAD(read);
  }
};
  

int sc_main(int, char*[] )
{
  channel a("a");
  mod_a modul_a("modul_a");
  mod_b modul_b("modul_b");
  modul_a.out(a);
  modul_b.input(a); 

  sc_start(120, SC_NS);
  return 0;
}
