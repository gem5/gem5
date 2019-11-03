/*****************************************************************************

  Licensed to Accellera Systems Initiative Inc. (Accellera) under one or
  more contributor license agreements.  See the NOTICE file distributed
  with this work for additional information regarding copyright ownership.
  Accellera licenses this file to you under the Apache License, Version 2.0
  (the "License"); you may not use this file except in compliance with the
  License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
  implied.  See the License for the specific language governing
  permissions and limitations under the License.

 *****************************************************************************/

/*****************************************************************************

  testbench.cpp -- 

  Original Author: Martin Janssen, Synopsys, Inc., 2002-02-15

 *****************************************************************************/

/*****************************************************************************

  MODIFICATION LOG - modifiers, enter your name, affiliation, date and
  changes you are making here.

      Name, Affiliation, Date:
  Description of Modification:

 *****************************************************************************/

/* Filename testbench.cc */
/* This is the implementation file for synchronous process `testbench' */

#include "testbench.h"

float gen_sample(float t);

extern float signal_freq;

void testbench::entry()
{
  float sample_val;
  float result_val;
  float time;

  FILE *data1 = fopen("Sample", "w");
  FILE *data2 = fopen("Result", "w");

  reset.write(true);
  wait(5);
  reset.write(false);
  wait();

  time = 0.0;
  while (true) {
    sample_val = gen_sample(time);
    sample.write(sample_val);
    wait();
    result_val = result.read();
    fprintf(data1, "%f\t%f\n", time, sample_val);
    fprintf(data2, "%f\t%f\n", time, result_val);
    char buf[BUFSIZ];
    sprintf( buf, "Input = %f\tOutput = %f", sample_val, result_val );
    cout << buf << endl;
    time += 1.0;
  }
} // end of entry function


float
gen_step(float t)
{
  if (t < 10.0)
    return (0.0);
  else 
    return (1.0);
}

float
gen_impulse(float t)
{
  if (t == 20.00)
    return (1.00);
  else 
    return (0.00);
}

float
gen_sine(float t, float freq) // freq in Hertz
{
  return sin(6.283185 * freq * t * CLOCK_PERIOD * 1e-9);
}

// This function actually generates the samples
float
gen_sample(float t)
{
 // return (gen_step(t));
  //  return (gen_impulse(t));
   return (gen_sine(t, signal_freq));
}
