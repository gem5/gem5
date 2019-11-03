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

  catch_actions.cpp -- Test for catch actions used with a custom report handler

  Original Author: Philipp A. Hartmann, Intel, 2017-01-22

 *****************************************************************************/

#include <systemc>

using std::cout;
using std::endl;
using sc_core::sc_actions;
using sc_core::SC_DISPLAY;
using sc_core::SC_ERROR;
using sc_core::SC_LOG;
using sc_core::SC_THROW;
using sc_core::sc_report;
using sc_core::sc_report_handler;

/* anonymous */ namespace {
bool last_report_thrown = false;
void custom_handler(const sc_report& rep, const sc_actions& actions)
{
    if ( actions & SC_DISPLAY ) {
        cout << endl
             // if last_report_thrown is set, we're in a catch action
             << (last_report_thrown ? "[caught] " : "[normal] ")
             << sc_core::sc_report_compose_message(rep)
             << endl;
    }

    // only log errors from catch actions
    if (rep.get_severity() == SC_ERROR) {
        if (actions & SC_LOG) {
            sc_assert( !(actions & SC_THROW) );
        } else {
            sc_assert( actions & SC_THROW );
        }
    }

    // cache SC_THROW state of current report
    last_report_thrown = (actions & SC_THROW);

    // delegate other actions to default handler
    sc_report_handler::default_handler(rep, actions & ~SC_DISPLAY);

} /* custom_handler */
} /* anonymous namespace */

int sc_main(int,char*[])
{
    // extended report logging, currently not checked in verify.pl
    sc_report_handler::set_log_file_name("catch_actions.ext.log");

    sc_report_handler::set_handler(custom_handler);
    SC_REPORT_INFO("catch_actions", "prepared custom handler");
    sc_assert(last_report_thrown == false);

    SC_REPORT_INFO("catch_actions", "preparing catch actions");
    sc_actions act = sc_report_handler::set_catch_actions(SC_DISPLAY | SC_LOG);
    sc_assert(act == SC_DISPLAY);
    sc_assert(sc_report_handler::get_catch_actions() == (SC_DISPLAY | SC_LOG));

    SC_REPORT_INFO("catch_actions", "only log errors from catch actions");
    act = sc_report_handler::set_actions
      (SC_ERROR, (sc_core::SC_DEFAULT_ERROR_ACTIONS & ~SC_LOG));

    // real test
    SC_REPORT_ERROR("catch_actions", "throwing an exception");

    // not reached
    return 0;
}
