
/*
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
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

/*
 * $Id$
 *
 */

#include "EventQueue.hh"
#include "Consumer.hh"

//static int global_counter = 0;

class TestConsumer1 : public Consumer {
public:
  TestConsumer1(int description) { m_description = description; }
  ~TestConsumer1() { }
  void wakeup() { cout << "Wakeup#1: " << m_description << endl; }
  // void wakeup() { global_counter++; }
  void print(ostream& out) const { out << "1:" << m_description << endl; }

private:
  int m_description;
};

class TestConsumer2 : public Consumer {
public:
  TestConsumer2(int description) { m_description = description; }
  ~TestConsumer2() { }
  void wakeup() { cout << "Wakeup#2: " << m_description << endl; }
  // void wakeup() { global_counter++; }
  void print(ostream& out) const { out << "2:" << m_description << endl; }
private:
  int m_description;
};

int main()
{
  EventQueue q;
  const int SIZE = 200;
  const int MAX_TIME = 10000;
  int numbers[SIZE];
  Consumer* consumers[SIZE];

  for (int i=0; i<SIZE; i++) {
    numbers[i] = random() % MAX_TIME;
    if (i%2 == 0) {
      consumers[i] = new TestConsumer1(i);
    } else {
      consumers[i] = new TestConsumer2(i);
    }
  }

  for(int i=0; i<SIZE; i++) {
    q.scheduleEvent(consumers[i], numbers[i]);
  }

  q.triggerEvents(MAX_TIME);

  for (int i=0; i<SIZE; i++) {
    delete consumers[i];
  }
}
