// example of a RISCV TLB that inherits from TLB
// helps in visualizing what the end product should look like


Addr buildKey(Addr vpn, uint16_t pcid)
{
    // Note ASID is 16 bits
    // The VPN in sv39 is up to 39-12=27 bits
    // The VPN in sv48 is up to 48-12=36 bits
    // The VPN in sv57 is up to 57-12=45 bits
    // So, shifting the ASID into the top 16 bits is safe.
    assert(bits(vpn, 63, 48) == 0);
    return (static_cast<Addr>(asid) << 48) | vpn;
}

TLBEntry * lookup(Addr key, uint16_t asid, BaseMMU::Mode mode, bool hidden) {



}
