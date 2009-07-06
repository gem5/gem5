/*
 * CreditLink_d.hh
 *
 * Niket Agarwal, Princeton University
 *
 * */
#ifndef CREDIT_LINK_D_H
#define CREDIT_LINK_D_H

#include "mem/ruby/network/garnet-fixed-pipeline/NetworkLink_d.hh"

class CreditLink_d : public NetworkLink_d {
public:
        CreditLink_d(int id, int link_latency, GarnetNetwork_d *net_ptr):NetworkLink_d(id, link_latency, net_ptr) {}
};

#endif
