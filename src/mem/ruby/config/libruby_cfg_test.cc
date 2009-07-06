
#include <iostream>
#include <assert.h>

#include "../libruby.hh"

int main(int argc, char* argv[])
{
  assert(argc == 2);
  const char* cfg_file = argv[1];

  libruby_init(cfg_file);
  libruby_print_config(std::cout);
}
