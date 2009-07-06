
struct ProcessorRequest {
  vector<CacheRequest*> cache_requests;
};

class ProcessorInterface {

public:

  void read_atomic(const Address & paddr, void* data, int len) {
    assert(paddr.getLineAddress() + RubyConfig::dataBlockBytes() >= paddr + len);
    // for now, atomics can't span two blocks.  Maybe fix this later
  }

  void read(const Address & paddr, const Address & rip, AccessModeType atype, void* data, const int len) {

    // create the CacheRequests
    ProcessorRequest* this_request = new ProcessorRequest;
    Address split_addr = paddr;
    int len_remaining = len;
    while (split_addr.getAddress() < paddr.getAddress() + len) {
      int split_len = (split_addr.getAddress() + len_remaining <= split_addr.getLineAddress() + RubyConfig::dataBlockBytes()) ?
        len_remaining :
        RubyConfig::dataBlockBytes() - split_addr.getOffset();
      CacheRequest creq = new CacheRequest( line_address(split_addr),
                                            split_addr,
                                            CacheRequestType_LD,
                                            rip,
                                            atype,
                                            split_len,
                                            PretchBit_No,
                                            laddr,
                                            0);  // SMT thread id);
      this_request->cache_requests.push_back(creq);
      split_addr += split_len;
      len_remaining -= split_len;
    }
    outstanding_requests.push_back(this_request);

  }

private:
  vector<ProcessorRequest*> outstanding_requests;
  Sequencer* m_sequencer;
};
