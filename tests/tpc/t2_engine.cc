#include <cassert>
#include <iostream>
#include "mem/cache/prefetch/t2_engine.hh"
using namespace gem5::prefetch;
int main() {
    T2DeltaRun r; r.last=1000;
    for (unsigned k=1;k<=16;++k) {
        assert(r.sample(1000+8*k,16,4));
        assert(r.equal==k);
        assert(r.state==(k==16?T2DeltaRun::Strided:T2DeltaRun::Observation));
    }
    T2DeltaRun reverse; reverse.last=1000;
    for (unsigned k=1;k<=16;++k) assert(reverse.sample(1000-8*k,16,4));
    assert(reverse.state==T2DeltaRun::Strided && reverse.delta==-8);
    T2DeltaRun unstable; unstable.last=1000;
    uint64_t a=1000;
    for (unsigned k=1;k<=5;++k) { a+=k; assert(unstable.sample(a,16,4)); }
    assert(unstable.state==T2DeltaRun::NonStrided && unstable.changing==4);
    T2DeltaRun zero; zero.last=123;
    for (unsigned k=0;k<16;++k) assert(zero.sample(123,16,4));
    assert(zero.delta==0 && zero.state==T2DeltaRun::Strided);
    int64_t delta;
    assert(t2Delta(2,10,delta) && delta==-8);
    assert(!t2Delta(UINT64_MAX,0,delta));
    uint64_t target;
    assert(t2Target(1000,-8,16,target) && target==872);
    assert(!t2Target(4,-8,1,target));
    assert(!t2Target(UINT64_MAX,8,1,target));
    assert(!t2Target(0,INT64_MIN,1,target));
    assert(t2Distance(101,10,0,64)==11);
    assert(t2Distance(100000,1,0,64)==64);
    assert(t2Distance(0,0,0,64)==1);
    assert(t2Distance(UINT64_MAX,2,1,64)==64);
    T2Mean mean(2); assert(!mean.add(0));
    assert(mean.add(10) && mean.add(20) && mean.value()==15);
    assert(mean.add(40) && mean.value()==30);
    mean.clear(); assert(mean.value()==0);
    T2Loop loop(16,16,16);
    assert(loop.branch(200,100,true,10) && !loop.recognized());
    assert(!loop.branch(200,100,true,30) && loop.recognized());
    assert(loop.count==1 && loop.loopPeriod()==20);
    assert(!loop.branch(150,120,true,40)); // interspersed branch inside interval
    assert(loop.pc==200 && loop.count==1);
    assert(!loop.branch(200,100,false,50)); // final not-taken is not a new edge
    assert(!loop.branch(200,100,true,60) && loop.loopPeriod()==25);
    assert(loop.branch(400,300,true,80) && loop.generation==2 && loop.count==0);
    assert(!loop.recognized() && loop.loopPeriod()==0);
    T2Loop bounded(1,0,2);
    assert(bounded.branch(1000,900,true,10));
    assert(!bounded.branch(1000,900,true,20));
    assert(bounded.branch(2000,1900,true,30));
    assert(!bounded.branch(1000,900,true,40) && bounded.pc==2000);
    assert(!bounded.branch(2000,1900,true,50));
    assert(bounded.branch(3000,2900,true,60));
    assert(bounded.branch(1000,900,true,70)); // FIFO replaced the old filter entry.
    T2Mean boundedMean(16);
    assert(!boundedMean.add(UINT64_MAX));
    std::cout << "T2_ENGINE_PASS\n";
}
