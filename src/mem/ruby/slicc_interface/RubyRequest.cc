#include <iostream>

#include "mem/ruby/slicc_interface/RubyRequest.hh"

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
