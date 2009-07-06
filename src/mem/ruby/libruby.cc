
#include <sys/wait.h>
#include <algorithm>

#include "mem/ruby/libruby_internal.hh"
#include "mem/ruby/system/RubyPort.hh"
#include "mem/ruby/system/System.hh"
#include "mem/ruby/eventqueue/RubyEventQueue.hh"
#include "mem/ruby/system/MemoryVector.hh"
#include "mem/ruby/common/Address.hh"

string RubyRequestType_to_string(const RubyRequestType& obj)
{
  switch(obj) {
  case RubyRequestType_IFETCH:
    return "IFETCH";
  case RubyRequestType_LD:
    return "LD";
  case RubyRequestType_ST:
    return "ST";
  case RubyRequestType_RMW:
    return "RMW";
  case RubyRequestType_NULL:
  default:
    assert(0);
    return "";
  }
}

RubyRequestType string_to_RubyRequestType(std::string str)
{
  if (str == "IFETCH")
    return RubyRequestType_IFETCH;
  else if (str == "LD")
    return RubyRequestType_LD;
  else if (str == "ST")
    return RubyRequestType_ST;
  else if (str == "RMW")
    return RubyRequestType_RMW;
  else
    assert(0);
  return RubyRequestType_NULL;
}

ostream& operator<<(ostream& out, const RubyRequestType& obj)
{
  cerr << "in op" << endl;
  out << RubyRequestType_to_string(obj);
  cerr << "flushing" << endl;
  out << flush;
  cerr << "done" << endl;
  return out;
}

vector<string> tokenizeString(string str, string delims)
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

void libruby_init(const char* cfg_filename)
{
  stringstream cfg_output;

  // first we execute the Ruby-lang configuration script
  int fd[2];
  int pid;
  if (pipe(fd) == -1) {
    perror("Error Creating Pipe");
    exit(EXIT_FAILURE);
  }

  pid = fork();
  if (pid == -1){
    perror("Error forking");
    exit(EXIT_FAILURE);
  }

  if (!pid) {
    // child
    close(fd[0]); // close the read end of the pipe
    // replace stdout with the write pipe
    if (dup2(fd[1], STDOUT_FILENO) == -1) {
      perror("Error redirecting stdout");
      exit(EXIT_FAILURE);
    }
#define QUOTE_MACRO(x, y) QUOTE_TXT(x,y)
#define QUOTE_TXT(x, y) #x y
    if (execlp("ruby", "ruby", "-I", QUOTE_MACRO(GEMS_ROOT, "/ruby/config"), QUOTE_MACRO(GEMS_ROOT, "/ruby/config/print_cfg.rb"), "-r", cfg_filename, NULL)) {
      perror("execlp");
      exit(EXIT_FAILURE);
    }
  } else {
    close(fd[1]);

    int child_status;
    if (wait(&child_status) == -1) {
      perror("wait");
      exit(EXIT_FAILURE);
    }
    if (child_status != EXIT_SUCCESS) {
      exit(EXIT_FAILURE);
    }

    char buf[100];
    int bytes_read;
    while( (bytes_read = read(fd[0], buf, 100)) > 0 ) {
      for (int i=0;i<bytes_read;i++) {
        //      cout << buf[i];
        cfg_output << buf[i];
      }
    }
    assert(bytes_read == 0);
    close(fd[0]);
  }

  vector<RubyObjConf> * sys_conf = new vector<RubyObjConf>;

  string line;
  getline(cfg_output, line) ;
  while ( !cfg_output.eof() ) {
    vector<string> tokens = tokenizeString(line, " ");
    assert(tokens.size() >= 2);
    vector<string> argv;
    for (size_t i=2; i<tokens.size(); i++) {
      std::replace(tokens[i].begin(), tokens[i].end(), '%', ' ');
      std::replace(tokens[i].begin(), tokens[i].end(), '#', '\n');
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

RubyPortHandle libruby_get_port(const char* port_name, void (*hit_callback)(int64_t access_id))
{
  return static_cast<RubyPortHandle>(RubySystem::getPort(port_name, hit_callback));
}

RubyPortHandle libruby_get_port_by_name(const char* port_name)
{
  return static_cast<RubyPortHandle>(RubySystem::getPortOnly(port_name));
}

void libruby_write_ram(uint64_t paddr, uint8_t* data, int len)
{
  RubySystem::getMemoryVector()->write(Address(paddr), data, len);
}

void libruby_read_ram(uint64_t paddr, uint8_t* data, int len)
{
  RubySystem::getMemoryVector()->read(Address(paddr), data, len);
}

int64_t libruby_issue_request(RubyPortHandle p, struct RubyRequest request)
{
  return static_cast<RubyPort*>(p)->makeRequest(request);
}

int libruby_tick(int n)
{
  RubySystem::getEventQueue()->triggerEvents(RubySystem::getEventQueue()->getTime() + n);
  return 0;
}

void libruby_destroy()
{
}

const char* libruby_last_error()
{
  return "";
}

void libruby_print_config(std::ostream & out)
{
  RubySystem::printConfig(out);
}

void libruby_print_stats(std::ostream & out)
{
  RubySystem::printStats(out);
}

uint64_t libruby_get_time() {
  return RubySystem::getCycleCount(0);
}
