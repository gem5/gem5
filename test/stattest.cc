/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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

#include <iomanip>
#include <iostream>
#include <string>
#include <unistd.h>

#include "base/cprintf.hh"
#include "base/misc.hh"
#include "base/statistics.hh"
#include "sim/host.hh"

using namespace std;
using namespace Statistics;

Tick curTick = 0;
Tick ticksPerSecond = ULL(2000000000);

Scalar<> s1;
Scalar<> s2;
Average<> s3;
Scalar<Counter, MainBin> s4;
Vector<Counter, MainBin> s5;
Distribution<Counter, MainBin> s6;
Vector<> s7;
AverageVector<> s8;
StandardDeviation<> s9;
AverageDeviation<> s10;
Scalar<Counter> s11;
Distribution<> s12;
VectorDistribution<> s13;
VectorStandardDeviation<> s14;
VectorAverageDeviation<> s15;
Vector2d<> s16;

Formula f1;
Formula f2;
Formula f3;
Formula f4;
Formula f5;
Formula f6;

MainBin bin1("bin1");
MainBin bin2("bin2");

double
testfunc()
{
    return 9.8;
}

class TestClass {
  public:
    double operator()() { return 9.7; }
};

char *progname = "";

void
usage()
{
    panic("incorrect usage.\n"
          "usage:\n"
          "\t%s [-v]\n", progname);
}

int
main(int argc, char *argv[])
{
    char c;
    progname = argv[0];
    PrintDescriptions = false;
    while ((c = getopt(argc, argv, "v")) != -1) {
        cprintf("c == %c\n", c);
        switch (c) {
          case 'v':
            PrintDescriptions = true;
            break;
          default:
            usage();
        }
    }

    s5.init(5);
    s6.init(1, 100, 13);
    s7.init(7);
    s8.init(10);
    s12.init(1, 100, 13);
    s13.init(4, 0, 99, 10);
    s14.init(9);
    s15.init(10);
    s16.init(2, 2);

    s1
        .name("Stat01")
        .desc("this is statistic 1")
        ;

    s2
        .name("Stat02")
        .desc("this is statistic 2")
        .prereq(s11)
        ;

    s3
        .name("Stat03")
        .desc("this is statistic 3")
        .prereq(s11)
        ;

    s4
        .name("Stat04")
        .desc("this is statistic 4")
        .prereq(s11)
        ;

    s5
        .name("Stat05")
        .desc("this is statistic 5")
        .prereq(s11)
        .subname(0, "foo1")
        .subname(1, "foo2")
        .subname(2, "foo3")
        .subname(3, "foo4")
        .subname(4, "foo5")
        ;

    s6
        .name("Stat06")
        .desc("this is statistic 6")
        .prereq(s11)
        ;

    s7
        .name("Stat07")
        .desc("this is statistic 7")
        .precision(1)
        .flags(pdf | total)
        .prereq(s11)
        ;

    s8
        .name("Stat08")
        .desc("this is statistic 8")
        .precision(2)
        .prereq(s11)
        .subname(4, "blarg")
        ;

    s9
        .name("Stat09")
        .desc("this is statistic 9")
        .precision(4)
        .prereq(s11)
        ;

    s10
        .name("Stat10")
        .desc("this is statistic 10")
        .prereq(s11)
        ;

    s12
        .name("Stat12")
        .desc("this is statistic 12")
        ;

    s13
        .name("Stat13")
        .desc("this is statistic 13")
        ;

    s14
        .name("Stat14")
        .desc("this is statistic 14")
        ;

    s15
        .name("Stat15")
        .desc("this is statistic 15")
        ;

    s16
        .name("Stat16")
        .desc("this is statistic 16")
        .flags(total)
        .subname(0, "sub0")
        .subname(1, "sub1")
        ;

    f1
        .name("Formula1")
        .desc("this is formula 1")
        .prereq(s11)
        ;

    f2
        .name("Formula2")
        .desc("this is formula 2")
        .prereq(s11)
        .precision(1)
        ;

    f3
        .name("Formula3")
        .desc("this is formula 3")
        .prereq(s11)
        .subname(0, "bar1")
        .subname(1, "bar2")
        .subname(2, "bar3")
        .subname(3, "bar4")
        .subname(4, "bar5")
        ;

    f4
        .name("Formula4")
        .desc("this is formula 4")
        ;

    f5
        .name("Formula5")
        .desc("this is formula 5")
        ;

    f6
        .name("Formula6")
        .desc("this is formula 6")
        ;

    check();

    bin1.activate();

    f1 = s1 + s2;
    f2 = (-s1) / (-s2) * -s3 + ULL(100) + s4;
    f3 = sum(s5) * s7;
    f4 = functor(testfunc);
    TestClass testclass;
    f5 = functor(testclass);
    f6 += constant(10.0);
    f6 += s5[3];

    s16[1][0] = 1;
    s16[0][1] = 3;
    s16[0][0] = 2;
    s16[1][1] = 9;
    s16[1][1] += 9;

    s11 = 1;
    s3 = 9;
    s8[3] = 9;
    s15[0].sample(1234);
    s15[1].sample(1234);
    s15[2].sample(1234);
    s15[3].sample(1234);
    s15[4].sample(1234);
    s15[5].sample(1234);
    s15[6].sample(1234);
    s15[7].sample(1234);
    s15[8].sample(1234);
    s15[9].sample(1234);

    s10.sample(1000000000);
    curTick += ULL(1000000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s10.sample(100000);
    s13[0].sample(12);
    s13[1].sample(29);
    s13[2].sample(12);
    s13[3].sample(29);
    s13[0].sample(42);
    s13[1].sample(29);
    s13[2].sample(42);
    s13[3].sample(32);
    s13[0].sample(52);
    s13[1].sample(49);
    s13[2].sample(42);
    s13[3].sample(25);
    s13[0].sample(32);
    s13[1].sample(49);
    s13[2].sample(22);
    s13[3].sample(49);
    s13[0].sample(62);
    s13[1].sample(99);
    s13[2].sample(72);
    s13[3].sample(23);
    s13[0].sample(52);
    s13[1].sample(78);
    s13[2].sample(69);
    s13[3].sample(49);

    s14[0].sample(1234);
    s14[1].sample(4134);
    s14[4].sample(1213);
    s14[3].sample(1124);
    s14[2].sample(1243);
    s14[7].sample(1244);
    s14[4].sample(7234);
    s14[2].sample(9234);
    s14[3].sample(1764);
    s14[7].sample(1564);
    s14[3].sample(3234);
    s14[1].sample(2234);
    s14[5].sample(1234);
    s14[2].sample(4334);
    s14[2].sample(1234);
    s14[4].sample(4334);
    s14[6].sample(1234);
    s14[8].sample(8734);
    s14[1].sample(5234);
    s14[3].sample(8234);
    s14[7].sample(5234);
    s14[4].sample(4434);
    s14[3].sample(7234);
    s14[2].sample(1934);
    s14[1].sample(9234);
    s14[5].sample(5634);
    s14[3].sample(1264);
    s14[7].sample(5223);
    s14[0].sample(1234);
    s14[0].sample(5434);
    s14[3].sample(8634);
    s14[1].sample(1234);


    s15[0].sample(1234);
    s15[1].sample(4134);
    curTick += ULL(1000000);
    s15[4].sample(1213);
    curTick += ULL(1000000);
    s15[3].sample(1124);
    curTick += ULL(1000000);
    s15[2].sample(1243);
    curTick += ULL(1000000);
    s15[7].sample(1244);
    curTick += ULL(1000000);
    s15[4].sample(7234);
    s15[2].sample(9234);
    s15[3].sample(1764);
    s15[7].sample(1564);
    s15[3].sample(3234);
    s15[1].sample(2234);
    curTick += ULL(1000000);
    s15[5].sample(1234);
    curTick += ULL(1000000);
    s15[9].sample(4334);
    curTick += ULL(1000000);
    s15[2].sample(1234);
    curTick += ULL(1000000);
    s15[4].sample(4334);
    s15[6].sample(1234);
    curTick += ULL(1000000);
    s15[8].sample(8734);
    curTick += ULL(1000000);
    s15[1].sample(5234);
    curTick += ULL(1000000);
    s15[3].sample(8234);
    curTick += ULL(1000000);
    s15[7].sample(5234);
    s15[4].sample(4434);
    s15[3].sample(7234);
    s15[2].sample(1934);
    s15[1].sample(9234);
    curTick += ULL(1000000);
    s15[5].sample(5634);
    s15[3].sample(1264);
    s15[7].sample(5223);
    s15[0].sample(1234);
    s15[0].sample(5434);
    s15[3].sample(8634);
    curTick += ULL(1000000);
    s15[1].sample(1234);

    s4 = curTick;

    s8[3] = 99999;

    s3 = 12;
    s3++;
    curTick += 9;

    s1 = 9;
    s1 += 9;
    s1 -= 11;
    s1++;
    ++s1;
    s1--;
    --s1;

    s2 = 9;

    s5[0] += 1;
    s5[1] += 2;
    s5[2] += 3;
    s5[3] += 4;
    s5[4] += 5;

    s7[0] = 10;
    s7[1] = 20;
    s7[2] = 30;
    s7[3] = 40;
    s7[4] = 50;
    s7[5] = 60;
    s7[6] = 70;

    s6.sample(0);
    s6.sample(1);
    s6.sample(2);
    s6.sample(3);
    s6.sample(4);
    s6.sample(5);
    s6.sample(6);
    s6.sample(7);
    s6.sample(8);
    s6.sample(9);

    bin2.activate();
    s6.sample(10);
    s6.sample(10);
    s6.sample(10);
    s6.sample(10);
    s6.sample(10);
    s6.sample(10);
    s6.sample(10);
    s6.sample(10);
    s6.sample(11);
    s6.sample(19);
    s6.sample(20);
    s6.sample(20);
    s6.sample(21);
    s6.sample(21);
    s6.sample(31);
    s6.sample(98);
    s6.sample(99);
    s6.sample(99);
    s6.sample(99);

    s9.sample(100);
    s9.sample(100);
    s9.sample(100);
    s9.sample(100);
    s9.sample(10);
    s9.sample(10);
    s9.sample(10);
    s9.sample(10);
    s9.sample(10);

    curTick += 9;
    s4 = curTick;
    s6.sample(100);
    s6.sample(100);
    s6.sample(100);
    s6.sample(101);
    s6.sample(102);

    s12.sample(100);

    bin1.activate();
    cout << "dump 1" << endl;
    dump(cout);
    cout << endl << endl;

    bin2.activate();
    cout << "dump 2" << endl;
    dump(cout);
    cout << endl << endl;

    cout << "dump 3" << endl;
    reset();
    dump(cout);
    cout << endl << endl;

    return 0;
}
