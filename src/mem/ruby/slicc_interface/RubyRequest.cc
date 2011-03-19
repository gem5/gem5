#include <iostream>

#include "mem/ruby/slicc_interface/RubyRequest.hh"

using namespace std;

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
