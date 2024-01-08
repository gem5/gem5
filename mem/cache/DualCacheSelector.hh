// src/mem/cache/DualCacheSelector.hh

#ifndef __MEM_CACHE_DUAL_CACHE_SELECTOR_HH__
#define __MEM_CACHE_DUAL_CACHE_SELECTOR_HH__

#include "base/addr_range.hh"
#include "base/types.hh"
#include "mem/port.hh"
#include "sim/sim_object.hh"

class DualCacheSelector : public SimObject
{
  public:
    typedef DualPort::SenderState SenderState;

    DualCacheSelector(const Params *p);

    MasterPort cpuPort;
    MasterPort cache1Port, cache2Port;

    bool cpuPortSendPacket(PacketPtr pkt);
    bool cache1PortRecvPacket(PacketPtr pkt);
    bool cache2PortRecvPacket(PacketPtr pkt);
    
    
    // 端口定义
    MasterPort& getMasterPort(const std::string &if_name, PortID idx = -1);
    SlavePort& getSlavePort(const std::string &if_name, PortID idx = -1);

    // 端口实现
    class MasterPort : public mem::MasterPort
    {
    public:
        MasterPort(const std::string& name, DualCacheSelector& owner) : 
            mem::MasterPort(name, owner) {}
        // ...其他必要的方法实现
    };

    class SlavePort : public mem::SlavePort
    {
    public:
        SlavePort(const std::string& name, DualCacheSelector& owner) : 
            mem::SlavePort(name, owner) {}
        // ...其他必要的方法实现
    };

    // 参数类
    struct Params : public SimObjectParams
    {
        // ...定义所需参数
    };

  private:
    // ...定义SimObject的内部数据和方法
};

#endif // __MEM_CACHE_DUAL_CACHE_SELECTOR_HH__
