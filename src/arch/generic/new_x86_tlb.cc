
// previously name concAddrPcid
Addr buildKey(Addr vpn, uint64_t pcid)
{
    return (vpn | pcid);
}

TLBEntry * lookup(Addr key, uint64_t pcid, BaseMMU::Mode mode, bool hidden) {

    lookup_key = buildKey(vpn, asid) // building the key
    TlbEntry *entry = NewTLB::lookup(lookup_key, mode, hidden) // calling the parent class
    return entry;

}