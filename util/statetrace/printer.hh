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

#ifndef PRINTER_HH
#define PRINTER_HH

#include <iostream>
#include <string>
#include <vector>

#include "refcnt.hh"

class TraceChild;
class PrinterObject;

typedef RefCountingPtr<PrinterObject> PrinterPointer;

class PrinterObject : public RefCounted
{
  protected:
    TraceChild * child;
  public:
    PrinterObject(TraceChild * newChild) : child(newChild)
    {;}
    virtual std::ostream & writeOut(std::ostream & os) = 0;
    virtual bool configure(std::string) = 0;
};

class NestingPrinter : public PrinterObject
{
  private:
    std::vector<std::string> constStrings;
    std::vector<PrinterPointer> printers;
    int switchVar;
    int numPrinters;
  public:
    NestingPrinter(TraceChild * newChild) :
        PrinterObject(newChild), numPrinters(0), switchVar(-1)
    {;}

    bool configure(std::string);

    std::ostream & writeOut(std::ostream & os);
};

class RegPrinter : public PrinterObject
{
  private:
    int intRegNum;
  public:
    RegPrinter(TraceChild * newChild, int num = 0) :
        PrinterObject(newChild), intRegNum(num)
    {;}

    void regNum(int num)
    {
        intRegNum = num;
    }

    int regNum()
    {
        return intRegNum;
    }

    bool configure(std::string);

    std::ostream & writeOut(std::ostream & os);
};

static inline std::ostream & operator << (std::ostream & os,
        PrinterObject & printer)
{
    return printer.writeOut(os);
}

static inline std::ostream & operator << (std::ostream & os,
        PrinterPointer & printer)
{
    return printer->writeOut(os);
}

#endif
