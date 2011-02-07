/*
 * Copyright (c) 2009 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/wait.h>
#include <algorithm>

#include "config/gems_root.hh"
#include "mem/ruby/common/Address.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/libruby_internal.hh"
#include "mem/ruby/recorder/Tracer.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/ruby/system/System.hh"

using namespace std;

string
RubyRequestType_to_string(const RubyRequestType& obj)
{
    switch(obj) {
      case RubyRequestType_IFETCH:
        return "IFETCH";
      case RubyRequestType_LD:
        return "LD";
      case RubyRequestType_ST:
        return "ST";
      case RubyRequestType_Load_Linked:
        return "Load_Linked";
      case RubyRequestType_Store_Conditional:
        return "Store_Conditional";
      case RubyRequestType_RMW_Read:
        return "RMW_Read";
      case RubyRequestType_RMW_Write:
        return "RMW_Write";
      case RubyRequestType_Locked_RMW_Read:
        return "Locked_RMW_Read";
      case RubyRequestType_Locked_RMW_Write:
        return "Locked_RMW_Write";
      case RubyRequestType_NULL:
      default:
        assert(0);
        return "";
    }
}

RubyRequestType
string_to_RubyRequestType(string str)
{
    if (str == "IFETCH")
        return RubyRequestType_IFETCH;
    else if (str == "LD")
        return RubyRequestType_LD;
    else if (str == "ST")
        return RubyRequestType_ST;
    else if (str == "Locked_Read")
        return RubyRequestType_Load_Linked;
    else if (str == "Locked_Write")
        return RubyRequestType_Store_Conditional;
    else if (str == "RMW_Read")
        return RubyRequestType_RMW_Read;
    else if (str == "RMW_Write")
        return RubyRequestType_RMW_Write;
    else if (str == "Locked_RMW_Read")
        return RubyRequestType_Locked_RMW_Read;
    else if (str == "Locked_RMW_Write")
        return RubyRequestType_Locked_RMW_Write;
    else
        assert(0);
    return RubyRequestType_NULL;
}

ostream&
operator<<(ostream& out, const RubyRequestType& obj)
{
    out << RubyRequestType_to_string(obj);
    out << flush;
    return out;
}

ostream&
operator<<(ostream& out, const RubyRequest& obj)
{
    out << hex << "0x" << obj.paddr << " data: 0x" << flush;
    for (int i = 0; i < obj.len; i++) {
        out << (int)obj.data[i];
    }
    out << dec << " type: " << RubyRequestType_to_string(obj.type) << endl;
    return out;
}

vector<string>
tokenizeString(string str, string delims)
{
    vector<string> tokens;
    char* pch;
    char* tmp;
    const char* c_delims = delims.c_str();
    tmp = new char[str.length()+1];
    strcpy(tmp, str.c_str());
    pch = strtok(tmp, c_delims);
    while (pch != NULL) {
        string tmp_str(pch);
        if (tmp_str == "null") tmp_str = "";
        tokens.push_back(tmp_str);

        pch = strtok(NULL, c_delims);
    }
    delete [] tmp;
    return tokens;
}

/*
 * The current state of M5/Ruby integration breaks the libruby
 * interface.  This code is ifdef'd out for now so that we can move
 * forward with the integration process for non-libruby uses.  We'll
 * have to go back and resolve the libruby compatibility issue at a
 * later date.
 */
#if 0
void
libruby_init(const char* cfg_filename)
{
    ifstream cfg_output(cfg_filename);

    vector<RubyObjConf> * sys_conf = new vector<RubyObjConf>;

    string line;
    getline(cfg_output, line) ;
    while ( !cfg_output.eof() ) {
        vector<string> tokens = tokenizeString(line, " ");
        assert(tokens.size() >= 2);
        vector<string> argv;
        for (size_t i=2; i<tokens.size(); i++) {
            replace(tokens[i].begin(), tokens[i].end(), '%', ' ');
            replace(tokens[i].begin(), tokens[i].end(), '#', '\n');
            argv.push_back(tokens[i]);
        }
        sys_conf->push_back(RubyObjConf(tokens[0], tokens[1], argv));
        tokens.clear();
        argv.clear();
        getline(cfg_output, line);
    }

    RubySystem::create(*sys_conf);
    delete sys_conf;
}
#endif

RubyPortHandle
libruby_get_port(const char* port_name,
                 void (*hit_callback)(int64_t access_id))
{
    //
    // Fix me: Hit callback is now a non-static member function pointer of
    // RubyPort and cannot be set to an arbitrary global function
    //
    return NULL;//static_cast<RubyPortHandle>(RubySystem::getPort(port_name, hit_callback));
}

RubyPortHandle libruby_get_port_by_name(const char* port_name)
{
    //
    // Fix me: Ports should now be initialized using the python configuration
    // system
    //
    return NULL;//static_cast<RubyPortHandle>(RubySystem::getPortOnly(port_name));
}

void
libruby_write_ram(uint64_t paddr, uint8_t* data, int len)
{
    RubySystem::getMemoryVector()->write(Address(paddr), data, len);
}

void
libruby_read_ram(uint64_t paddr, uint8_t* data, int len)
{
    RubySystem::getMemoryVector()->read(Address(paddr), data, len);
}

int64_t
libruby_issue_request(RubyPortHandle p, struct RubyRequest request)
{
    //
    // Fix me: Ports should now be accessed using the python configuration
    // system
    //
    return 0;//return static_cast<RubyPort*>(p)->makeRequest(request);
}

int
libruby_tick(int n)
{
    RubyEventQueue *eventq = RubySystem::getEventQueue();
    eventq->triggerEvents(eventq->getTime() + n);
    return 0;
}

void
libruby_destroy()
{
}

const char*
libruby_last_error()
{
    return "";
}

void
libruby_print_config(ostream & out)
{
    RubySystem::printConfig(out);
}

void
libruby_print_stats(ostream & out)
{
    RubySystem::printStats(out);
}
void
libruby_playback_trace(char * trace_filename)
{
    RubySystem::getTracer()->playbackTrace(trace_filename);
}

void
libruby_start_tracing(char * record_filename)
{
    // start the trace
    RubySystem::getTracer()->startTrace(record_filename);
}

void
libruby_stop_tracing()
{
    // start the trace
    RubySystem::getTracer()->stopTrace();
}

uint64_t
libruby_get_time()
{
    return RubySystem::getCycleCount(0);
}
