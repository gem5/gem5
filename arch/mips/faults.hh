/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __ALPHA_FAULTS_HH__
#define __ALPHA_FAULTS_HH__

#include "sim/faults.hh"
#include "arch/isa_traits.hh" //For the Addr type

class Garbage;
class Fault;

class AlphaFault : public Fault
{
  public:
    AlphaFault(char * newName, int newId, Addr newVect)
        : Fault(newName, newId), vect(newVect)
    {;}

    Addr vect;
};

extern class ResetFaultType : public AlphaFault
{
  public:
    ResetFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const ResetFault;

extern class ArithmeticFaultType : public AlphaFault
{
  public:
    ArithmeticFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const ArithmeticFault;

extern class InterruptFaultType : public AlphaFault
{
  public:
    InterruptFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const InterruptFault;

extern class NDtbMissFaultType : public AlphaFault
{
  public:
    NDtbMissFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const NDtbMissFault;

extern class PDtbMissFaultType : public AlphaFault
{
  public:
    PDtbMissFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const PDtbMissFault;

extern class DtbPageFaultType : public AlphaFault
{
  public:
    DtbPageFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const DtbPageFault;

extern class DtbAcvFaultType : public AlphaFault
{
  public:
    DtbAcvFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const DtbAcvFault;

extern class ItbMissFaultType : public AlphaFault
{
  public:
    ItbMissFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const ItbMissFault;

extern class ItbPageFaultType : public AlphaFault
{
  public:
    ItbPageFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const ItbPageFault;

extern class ItbAcvFaultType : public AlphaFault
{
  public:
    ItbAcvFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const ItbAcvFault;

extern class UnimplementedOpcodeFaultType : public AlphaFault
{
  public:
    UnimplementedOpcodeFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const UnimplementedOpcodeFault;

extern class FloatEnableFaultType : public AlphaFault
{
  public:
    FloatEnableFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const FloatEnableFault;

extern class PalFaultType : public AlphaFault
{
  public:
    PalFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const PalFault;

extern class IntegerOverflowFaultType : public AlphaFault
{
  public:
    IntegerOverflowFaultType(char * newName, int newId, Addr newVect)
        : AlphaFault(newName, newId, newVect)
    {;}
} * const IntegerOverflowFault;

extern Fault ** ListOfFaults[];
extern int NumFaults;

#endif // __FAULTS_HH__
