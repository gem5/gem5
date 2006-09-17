/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 *
 * Authors: Gabe Black
 */

#include "tracechild.hh"
#include "printer.hh"

using namespace std;

//Types of printers. If none is found, or there is an error in the input,
//there are psuedo types to return.
enum PrinterType {PRINTER_NONE, PRINTER_ERROR, PRINTER_NESTING, PRINTER_REG};

int findEndOfRegPrinter(string, int);
int findEndOfNestingPrinter(string, int);
PrinterType findSub(string, int &, int &);

//This is pretty easy. Just find the closing parenthesis.
int findEndOfRegPrinter(string config, int startPos)
{
    int pos = config.find(")", startPos);
    if(pos == string::npos)
    {
        cerr << "Couldn't find the closing parenthesis for a reg printer" << endl;
        return 0;
    }
    return pos;
}

//This is a little harder. We need to make sure we don't
//grab an ending parenthesis that belongs to the nesting printer.
int findEndOfNestingPrinter(string config, int startPos)
{
    int length = config.length();
    int pos = startPos;
    int endPos = length;
    int parenPos = config.find(")", pos);
    //If we didn't find an ending parenthesis at all, we're in trouble
    if(parenPos == string::npos)
    {
        cerr << "Couldn't find the closing parenthesis for a nesting printer on the first try" << endl;
        return 0;
    }
    //Keep pulling out embedded stuff until we can't any more
    //we need to make sure we aren't skipping over the parenthesis
    //that ends -this- printer.
    PrinterType type = findSub(config, pos, endPos);
    if(type == PRINTER_ERROR)
        return 0;
    while(type != PRINTER_NONE && endPos >= parenPos)
    {
        //Find the next closest ending parenthesis since we passed
        //up the last one
        parenPos = config.find(")", endPos + 1);
        //If we didn't find one, we're in trouble
        if(parenPos == string::npos)
        {
            cerr << "Couldn't find the closing parenthesis for a nested printer on later tries" << endl;
            return 0;
        }
        //Start looking for the end of this printer and embedded
        //stuff past the one we just found
        pos = endPos + 1;
        //Reset endPos so we search to the end of config
        endPos = length;
        type = findSub(config, pos, endPos);
        if(type == PRINTER_ERROR)
            return 0;
    }
    //We ran out of embedded items, and we didn't pass up our last
    //closing paren. This must be the end of this printer.
    return parenPos;
}

//Find a sub printer. This looks for things which have a type defining
//character and then an opening parenthesis. The type is returned, and
//startPos and endPos are set to the beginning and end of the sub printer
//On entry, the search starts at index startPos and ends at either index
//endPos or a closing parenthesis, whichever comes first
PrinterType findSub(string config, int & startPos, int & endPos)
{
    int length = config.length();
    //Figure out where the different types of sub printers may start
    int regPos = config.find("%(", startPos);
    int nestingPos = config.find("~(", startPos);
    //If a type of printer wasn't found, say it was found too far away.
    //This simplifies things later
    if(regPos == string::npos)
        regPos = endPos;
    if(nestingPos == string::npos)
        nestingPos = endPos;
    //If we find a closing paren, that marks the
    //end of the region we're searching.
    int closingPos = config.find(")", startPos);
    if(closingPos != string::npos &&
            closingPos < regPos &&
            closingPos < nestingPos)
        return PRINTER_NONE;
    //If we didn't find anything close enough, say so.
    if(regPos >= endPos && nestingPos >= endPos)
        return PRINTER_NONE;
    //At this point, we know that one of the options starts legally
    //We need to find which one is first and return that
    if(regPos < nestingPos)
    {
        int regEnd = findEndOfRegPrinter(config, regPos + 2);
        //If we couldn't find the end...
        if(!regEnd)
        {
            cerr << "Couldn't find the end of the reg printer" << endl;
            return PRINTER_ERROR;
        }
        //Report the sub printer's vitals.
        startPos = regPos;
        endPos = regEnd;
        return PRINTER_REG;
    }
    else
    {
        int nestingEnd = findEndOfNestingPrinter(config, nestingPos + 2);
        //If we couldn't find the end...
        if(!nestingEnd)
        {
            cerr << "Couldn't find the end of the nesting printer" << endl;
            return PRINTER_ERROR;
        }
        //Report the sub printer's vitals.
        startPos = nestingPos;
        endPos = nestingEnd;
        return PRINTER_NESTING;
    }
    return PRINTER_NONE;
}

//Set up a nesting printer. This printer can contain sub printers
bool NestingPrinter::configure(string config)
{
    //Clear out any old stuff
    constStrings.clear();
    numPrinters = 0;
    printers.clear();
    int length = config.length();
    int startPos = 0, endPos = length;
    int lastEndPos = -1;
    //Try to find a sub printer
    PrinterType type = findSub(config, startPos, endPos);
    if(type == PRINTER_ERROR)
    {
        cerr << "Problem finding first sub printer" << endl;
        return false;
    }
    while(type != PRINTER_NONE)
    {
        string prefix = config.substr(lastEndPos + 1, startPos - lastEndPos - 1);
        lastEndPos = endPos;
        constStrings.push_back(prefix);
        string subConfig, subString;
        int commaPos, lastCommaPos, childSwitchVar;
        switch(type)
        {
            //If we found a plain register printer
          case PRINTER_REG:
            numPrinters++;
            //Get the register name
            subConfig = config.substr(startPos + 2, endPos - startPos - 2);
            //Set up the register printer
            RegPrinter * regPrinter = new RegPrinter(child);
            if(!regPrinter->configure(subConfig))
            {
                delete regPrinter;
                cerr << "Error configuring reg printer" << endl;
                return false;
            }
            printers.push_back(regPrinter);
            break;
            //If we found an embedded nesting printer
          case PRINTER_NESTING:
            numPrinters++;
            //Punt on reading in all the parameters of the nesting printer
            NestingPrinter * nestingPrinter = new NestingPrinter(child);
            subConfig = config.substr(startPos + 2, endPos - startPos - 2);
            lastCommaPos = string::npos;
            commaPos = subConfig.find(",");
            if(commaPos == string::npos)
                return false;
            childSwitchVar = child->getRegNum(subConfig.substr(0, commaPos));
            if(childSwitchVar == -1)
            {
                cerr << "Couldn't configure switching variable!" << endl;
                return false;
            }
            //Eat up remaining arguments
            while(commaPos != string::npos)
            {
                lastCommaPos = commaPos;
                commaPos = subConfig.find(",", commaPos + 1);
            }
            if(lastCommaPos != string::npos)
            {
                subConfig = subConfig.substr(lastCommaPos + 1, subConfig.length() - lastCommaPos - 1);
            }
            if(!nestingPrinter->configure(subConfig))
            {
                delete nestingPrinter;
                cerr << "Error configuring nesting printer" << endl;
                return false;
            }
            nestingPrinter->switchVar = childSwitchVar;
            printers.push_back(nestingPrinter);
            break;
          default:
            cerr << "Unrecognized printer type" << endl;
            return false;
        }
        //Move down past what we just parsed
        startPos = endPos + 1;
        endPos = length;
        type = findSub(config, startPos, endPos);
        if(type == PRINTER_ERROR)
        {
            cerr << "Unable to find subprinters on later tries" << endl;
            return false;
        }
    }
    //Put in the trailing stuff
    string trailer = config.substr(startPos, length - startPos);
    constStrings.push_back(trailer);
    return true;
}

bool RegPrinter::configure(string config)
{
    //Figure out what our register number is based on the name we're given
    int num = child->getRegNum(config);
    if(num == -1)
    {
        cerr << "Couldn't find register " << config << endl;
        return false;
    }
    regNum(num);
    return true;
}

ostream & NestingPrinter::writeOut(ostream & os)
{
    if(switchVar == -1 || child->diffSinceUpdate(switchVar))
    {
        int x;
        for(x = 0; x < numPrinters; x++)
        {
            os << constStrings[x];
            os << printers[x];
        }
        os << constStrings[x];
    }
    return os;
}

ostream & RegPrinter::writeOut(ostream & os)
{
    os << child->printReg(intRegNum);
    return os;
}

