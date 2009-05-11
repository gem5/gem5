/*
 * CreditLink_d.h
 *
 * Niket Agarwal, Princeton University
 *
 * */
#ifndef CREDIT_LINK_D_H
#define CREDIT_LINK_D_H

#include "NetworkLink_d.hh"

class CreditLink_d : public NetworkLink_d {
public:
        CreditLink_d(int id):NetworkLink_d(id) {}
};

#endif
