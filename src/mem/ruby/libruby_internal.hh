#ifndef LIBRUBY_INTERNAL_H
#define LIBRUBY_INTERNAL_H

#include "mem/ruby/libruby.hh"

#include <ostream>
#include <string>

std::string RubyRequestType_to_string(const RubyRequestType& obj);
RubyRequestType string_to_RubyRequestType(std::string);
std::ostream& operator<<(std::ostream& out, const RubyRequestType& obj);

#endif
