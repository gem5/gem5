/* Copyright (c) 2012 Massachusetts Institute of Technology
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <fstream>

#include "Assert.h"
#include "Config.h"

using namespace std;

namespace LibUtil
{
    void readFile(const char *filename_, map<String, String> &config)
    {
        std::ifstream ist_(filename_);

        // Set a Config from ist_
        // Read in keys and values, keeping internal whitespace
        typedef String::size_type pos;
        const String& delimiter = "=";
        const String& comment = "#";
        const String& sentry = "End";
        const pos skip = delimiter.length();        // length of separator

        String nextline = "";  // might need to read ahead to see where value ends

        while(ist_ || nextline.length() > 0)
        {
            // Read an entire line at a time
            String line;
            if(nextline.length() > 0)
            {
                line = nextline;  // we read ahead; use it now
                nextline = "";
            }
            else
            {
                //std::getline(ist_, line);
                safeGetline(ist_, line);
            }

            // Ignore comments and the spaces on both ends
            line = line.substr(0, line.find(comment));
            line.trim();

            // Check for end of file sentry
            if((sentry != "") && (line.find(sentry) != String::npos)) return;

            if(line.length() == 0)
                continue;

            // Parse the line if it contains a delimiter
            pos delimPos = line.find(delimiter);
            ASSERT((delimPos < String::npos), "Invalid config line: '" + line + "'");

            // Extract the key
            String key = line.substr(0, delimPos);
            line.replace(0, delimPos+skip, "");

            // See if value continues on the next line
            // Stop at blank line, next line with a key, end of stream,
            // or end of file sentry
            bool terminate = false;
            while(!terminate && ist_)
            {
                if(line.at(line.size() - 1) == '\\')
                    line.erase(line.size() - 1);
                else
                    break;

                //std::getline(ist_, nextline);
                safeGetline(ist_, nextline);
                terminate = true;

                String nlcopy = nextline;
                nlcopy.trim();
                if(nlcopy == "") continue;

                nextline = nextline.substr(0, nextline.find(comment));
                //if(nextline.find(delim) != String::npos)
                //    continue;
                if((sentry != "") && (nextline.find(sentry) != String::npos))
                    continue;

                //nlcopy = nextline;
                //nlcopy.trim();
                //if(nlcopy != "") line += "\n";
                line += nextline;
                nextline = "";
                terminate = false;
            }

            // Store key and value
            key.trim();
            line.trim();
            config[key] = line;  // overwrites if key is repeated
        }
    }
}

